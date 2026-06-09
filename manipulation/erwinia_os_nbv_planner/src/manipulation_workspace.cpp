#include "erwinia_os_nbv_planner/manipulation_workspace.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>
#include <chrono>
#include <thread>

namespace erwinia_os_nbv_planner
{

    ManipulationWorkspace::ManipulationWorkspace(
        const std::shared_ptr<MoveItInterface> &moveit_interface,
        const std::shared_ptr<NBVVisualizer> &visualizer)
        : moveit_interface_(moveit_interface),
          visualizer_(visualizer)
    {
        if (!moveit_interface_)
        {
            throw std::runtime_error("MoveItInterface pointer is null");
        }
    }

    uint64_t ManipulationWorkspace::poseToVoxelKey(const geometry_msgs::msg::Pose &pose) const
    {
        // Discretize the position to voxel coordinates
        int32_t x = discretizeCoordinate(pose.position.x);
        int32_t y = discretizeCoordinate(pose.position.y);
        int32_t z = discretizeCoordinate(pose.position.z);

        // Pack into 64-bit key: 21 bits each for x,y,z (supports range of ~±1000 meters at 1mm resolution)
        uint64_t key = 0;
        key |= (static_cast<uint64_t>(x & 0x1FFFFF) << 42);
        key |= (static_cast<uint64_t>(y & 0x1FFFFF) << 21);
        key |= (static_cast<uint64_t>(z & 0x1FFFFF));

        return key;
    }

    geometry_msgs::msg::Pose ManipulationWorkspace::voxelKeyToPose(uint64_t key) const
    {
        // Unpack the voxel coordinates
        int32_t x = static_cast<int32_t>((key >> 42) & 0x1FFFFF);
        int32_t y = static_cast<int32_t>((key >> 21) & 0x1FFFFF);
        int32_t z = static_cast<int32_t>(key & 0x1FFFFF);

        // Handle sign extension for 21-bit signed integers
        if (x & 0x100000) x |= 0xFFE00000;
        if (y & 0x100000) y |= 0xFFE00000;
        if (z & 0x100000) z |= 0xFFE00000;

        // Convert back to continuous coordinates (center of voxel)
        geometry_msgs::msg::Pose pose;
        pose.position.x = (x + 0.5) * voxel_size_;
        pose.position.y = (y + 0.5) * voxel_size_;
        pose.position.z = (z + 0.5) * voxel_size_;

        // Default orientation
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;

        return pose;
    }

    int32_t ManipulationWorkspace::discretizeCoordinate(double coord) const
    {
        return static_cast<int32_t>(std::floor(coord / voxel_size_));
    }

    bool ManipulationWorkspace::sampleRandomReachablePose(geometry_msgs::msg::Pose &pose_out)
    {
        if (!moveit_interface_)
        {
            return false;
        }

        // Get joint model group to understand joint limits
        moveit::core::JointModelGroupConstPtr jmg;
        if (!moveit_interface_->getJointModelGroup(jmg))
        {
            return false;
        }

        // Get joint names and bounds
        const std::vector<std::string> &joint_names = jmg->getActiveJointModelNames();
        std::vector<double> random_joint_positions(joint_names.size());

        // Random number generator (fixed seed for reproducibility)
        static std::mt19937 gen(42u);

        // Sample random joint positions within bounds
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            const moveit::core::JointModel *joint = jmg->getJointModel(joint_names[i]);
            const moveit::core::JointModel::Bounds &bounds = joint->getVariableBounds();

            if (!bounds.empty())
            {
                double min_bound = bounds[0].min_position_;
                double max_bound = bounds[0].max_position_;
                std::uniform_real_distribution<> dis(min_bound, max_bound);
                random_joint_positions[i] = dis(gen);
            }
            else
            {
                random_joint_positions[i] = 0.0;
            }
        }

        // Check if the state is valid (collision-free)
        if (!moveit_interface_->isStateValid(random_joint_positions))
        {
            return false;
        }

        // Compute forward kinematics to get end-effector pose
        std::string ee_link = moveit_interface_->getEndEffectorLink();
        if (ee_link.empty())
        {
            return false;
        }

        // We need to get the pose for this joint configuration
        // Create a temporary robot state and compute FK
        auto psm = moveit_interface_->getPlanningSceneMonitor();
        if (!psm)
        {
            return false;
        }
        
        planning_scene_monitor::LockedPlanningSceneRO scene(psm);
        moveit::core::RobotState state = scene->getCurrentState();
        state.setJointGroupPositions(jmg.get(), random_joint_positions);
        state.update();

        const Eigen::Isometry3d &ee_transform = state.getGlobalLinkTransform(ee_link);
        pose_out = tf2::toMsg(ee_transform);

        return true;
    }

    bool ManipulationWorkspace::learnWorkspace(int num_samples, bool visualize)
    {
        // Clear existing workspace
        reachable_voxels_.clear();

        if (!moveit_interface_)
        {
            return false;
        }

        // Check if visualization is requested but visualizer is not available
        if (visualize && !visualizer_)
        {
            RCLCPP_WARN(rclcpp::get_logger("manipulation_workspace"),
                       "Visualization requested but no visualizer provided. Continuing without visualization.");
            visualize = false;
        }

        // Clear previous visualizations if needed
        if (visualize && visualizer_)
        {
            visualizer_->clearAll("workspace_learning");
        }

        int valid_samples = 0;
        std::vector<octomap::point3d> visualization_points;
        const int batch_size = 100; // Visualize in batches for better performance
        int batch_id = 0; // Track batch number for unique marker IDs

        RCLCPP_INFO(rclcpp::get_logger("manipulation_workspace"),
                   "Learning workspace with %d samples%s",
                   num_samples, visualize ? " (with visualization)" : "");

        for (int i = 0; i < num_samples; ++i)
        {
            geometry_msgs::msg::Pose pose;
            if (sampleRandomReachablePose(pose))
            {
                // Convert pose to voxel key and add to set
                uint64_t voxel_key = poseToVoxelKey(pose);
                reachable_voxels_.insert(voxel_key);
                
                // Collect point for visualization
                if (visualize && visualizer_)
                {
                    octomap::point3d point(pose.position.x, pose.position.y, pose.position.z);
                    visualization_points.push_back(point);
                    
                    // Publish in batches for better performance
                    if (visualization_points.size() >= batch_size)
                    {
                        visualizer_->publishPoints(visualization_points, voxel_size_ * 0.5, std_msgs::msg::ColorRGBA(), 0.8f, "workspace_learning_" + std::to_string(batch_id));
                        visualization_points.clear();
                        batch_id++;
                    }
                }
                
                valid_samples++;
            }

            // Progress indicator every 1000 samples
            if ((i + 1) % 1000 == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("manipulation_workspace"),
                           "Sampled %d/%d configurations, found %d valid poses in %zu voxels",
                           i + 1, num_samples, valid_samples, reachable_voxels_.size());
            }
        }
        
        // Publish any remaining points
        if (visualize && visualizer_ && !visualization_points.empty())
        {
            visualizer_->publishPoints(visualization_points, voxel_size_ * 0.5, std_msgs::msg::ColorRGBA(), 0.8f, "workspace_learning_" + std::to_string(batch_id));
        }

        RCLCPP_INFO(rclcpp::get_logger("manipulation_workspace"),
                   "Workspace learning complete: %zu reachable voxels from %d valid samples",
                   reachable_voxels_.size(), valid_samples);

        // Fit bounding sphere around learned workspace
        if (!reachable_voxels_.empty())
        {
            fitBoundingSphere();
        }

        return !reachable_voxels_.empty();
    }

    bool ManipulationWorkspace::isPoseReachable(const geometry_msgs::msg::Pose &pose) const
    {
        uint64_t voxel_key = poseToVoxelKey(pose);
        return reachable_voxels_.find(voxel_key) != reachable_voxels_.end();
    }

    bool ManipulationWorkspace::saveWorkspaceToFile(const std::string &file_path) const
    {
        std::ofstream file(file_path, std::ios::binary);
        if (!file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("manipulation_workspace"),
                        "Failed to open file for writing: %s", file_path.c_str());
            return false;
        }

        // Write voxel size
        file.write(reinterpret_cast<const char*>(&voxel_size_), sizeof(voxel_size_));

        // Write number of voxels
        size_t num_voxels = reachable_voxels_.size();
        file.write(reinterpret_cast<const char*>(&num_voxels), sizeof(num_voxels));

        // Write voxel keys
        for (uint64_t key : reachable_voxels_)
        {
            file.write(reinterpret_cast<const char*>(&key), sizeof(key));
        }

        file.close();

        RCLCPP_INFO(rclcpp::get_logger("manipulation_workspace"),
                   "Saved workspace with %zu voxels to %s", num_voxels, file_path.c_str());

        return true;
    }

    bool ManipulationWorkspace::loadWorkspaceFromFile(const std::string &file_path)
    {
        std::ifstream file(file_path, std::ios::binary);
        if (!file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("manipulation_workspace"),
                        "Failed to open file for reading: %s", file_path.c_str());
            return false;
        }

        // Read voxel size
        file.read(reinterpret_cast<char*>(&voxel_size_), sizeof(voxel_size_));

        // Read number of voxels
        size_t num_voxels;
        file.read(reinterpret_cast<char*>(&num_voxels), sizeof(num_voxels));

        // Clear existing data and read voxel keys
        reachable_voxels_.clear();
        for (size_t i = 0; i < num_voxels; ++i)
        {
            uint64_t key;
            file.read(reinterpret_cast<char*>(&key), sizeof(key));
            reachable_voxels_.insert(key);
        }

        file.close();

        RCLCPP_INFO(rclcpp::get_logger("manipulation_workspace"),
                   "Loaded workspace with %zu voxels from %s", reachable_voxels_.size(), file_path.c_str());

        // Fit bounding sphere around loaded workspace
        if (!reachable_voxels_.empty())
        {
            fitBoundingSphere();
        }

        return !reachable_voxels_.empty();
    }

    void ManipulationWorkspace::fitBoundingSphere()
    {
        if (reachable_voxels_.empty())
        {
            sphere_center_ = Eigen::Vector3d::Zero();
            sphere_radius_ = 0.0;
            return;
        }

        // Compute centroid of all voxel centers
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (uint64_t key : reachable_voxels_)
        {
            geometry_msgs::msg::Pose pose = voxelKeyToPose(key);
            sum.x() += pose.position.x;
            sum.y() += pose.position.y;
            sum.z() += pose.position.z;
        }
        sphere_center_ = sum / static_cast<double>(reachable_voxels_.size());

        // Find maximum distance from centroid to any voxel
        double max_dist_sq = 0.0;
        for (uint64_t key : reachable_voxels_)
        {
            geometry_msgs::msg::Pose pose = voxelKeyToPose(key);
            Eigen::Vector3d point(pose.position.x, pose.position.y, pose.position.z);
            double dist_sq = (point - sphere_center_).squaredNorm();
            if (dist_sq > max_dist_sq)
            {
                max_dist_sq = dist_sq;
            }
        }
        sphere_radius_ = std::sqrt(max_dist_sq);

        RCLCPP_INFO(rclcpp::get_logger("manipulation_workspace"),
                   "Bounding sphere fitted: center=[%.3f, %.3f, %.3f], radius=%.3f m",
                   sphere_center_.x(), sphere_center_.y(), sphere_center_.z(), sphere_radius_);
    }

    double ManipulationWorkspace::getDistance(const Eigen::Vector3d &point) const
    {
        // Compute distance from point to sphere center
        double dist_to_center = (point - sphere_center_).norm();
        
        // Signed distance: positive outside, negative inside
        return dist_to_center - sphere_radius_;
    }

}