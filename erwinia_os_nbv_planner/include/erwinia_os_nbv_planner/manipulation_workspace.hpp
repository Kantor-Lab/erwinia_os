#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <vector>
#include <unordered_set>
#include <fstream>
#include <random>

#include "erwinia_os_nbv_planner/moveit_interface.hpp"
#include "erwinia_os_nbv_planner/nbv_visualizer.hpp"

namespace erwinia_os_nbv_planner
{

    class ManipulationWorkspace
    {
    public:
        /**
         * @brief Constructor for ManipulationWorkspace
         * @param moveit_interface Shared pointer to MoveItInterface for motion planning and validation
         * @param visualizer Optional shared pointer to NBVVisualizer for visualization during learning
         * @throws std::runtime_error if moveit_interface is null
         */
        ManipulationWorkspace(
            const std::shared_ptr<MoveItInterface> &moveit_interface,
            const std::shared_ptr<NBVVisualizer> &visualizer = nullptr);

        /**
         * @brief Sample a random collision-free joint configuration and compute end-effector pose
         * 
         * Generates random joint values within joint limits, validates the configuration for
         * collisions, and if valid, computes the corresponding end-effector pose using forward
         * kinematics.
         * 
         * @param pose_out Output parameter containing the end-effector pose if sampling succeeds
         * @return true if a valid collision-free configuration was found, false otherwise
         */
        bool sampleRandomReachablePose(geometry_msgs::msg::Pose &pose_out);

        /**
         * @brief Learn the reachable workspace by sampling random configurations
         * 
         * Samples random joint configurations, validates each for collisions,
         * computes forward kinematics, and discretizes reachable end-effector positions
         * into voxels. The resulting voxel set represents the reachable workspace.
         * Progress is logged every 1000 samples.
         * 
         * @param num_samples Number of random configurations to sample (default: 10000)
         * @param visualize If true, visualize each valid end-effector position in RViz (default: false)
         * @return true if at least one reachable voxel was found, false otherwise
         */
        bool learnWorkspace(int num_samples = 10000, bool visualize = false);

        /**
         * @brief Check if a given pose is within the learned reachable workspace
         * 
         * Discretizes the input pose position into a voxel key and checks if that
         * voxel exists in the learned workspace. This provides a fast O(1) lookup.
         * Note: Only position is considered, not orientation.
         * 
         * @param pose The pose to check for reachability
         * @return true if the pose's position falls within a reachable voxel, false otherwise
         */
        bool isPoseReachable(const geometry_msgs::msg::Pose &pose) const;

        /**
         * @brief Save the learned workspace to a binary file
         * 
         * Saves the voxel size and all reachable voxel keys to a binary file for
         * later loading. This allows the expensive learning process to be done once
         * and reused across multiple runs.
         * 
         * File format: [voxel_size (8 bytes)][num_voxels (8 bytes)][voxel_key_1][voxel_key_2]...
         * 
         * @param file_path Path to the output file
         * @return true if save succeeded, false if file could not be opened
         */
        bool saveWorkspaceToFile(const std::string &file_path) const;

        /**
         * @brief Load a previously saved workspace from a binary file
         * 
         * Loads the voxel size and reachable voxel keys from a binary file created
         * by saveWorkspaceToFile(). Clears any existing workspace data before loading.
         * 
         * @param file_path Path to the input file
         * @return true if load succeeded and workspace is non-empty, false if file could not be opened
         */
        bool loadWorkspaceFromFile(const std::string &file_path);

        /**
         * @brief Get the current voxel size in meters
         * @return Voxel size (default: 0.02m = 2cm)
         */
        double getVoxelSize() const { return voxel_size_; }
        
        /**
         * @brief Set the voxel size for workspace discretization
         * @param size Voxel size in meters (smaller = higher resolution but more memory)
         */
        void setVoxelSize(double size) { voxel_size_ = size; }

        /**
         * @brief Get the number of unique reachable voxels in the workspace
         * @return Count of reachable voxels
         */
        size_t getNumReachableVoxels() const { return reachable_voxels_.size(); }

        /**
         * @brief Get signed distance from a point to the workspace bounding sphere
         * 
         * Returns the signed distance from the given point to the surface of the bounding sphere:
         * - Positive distance: point is outside the sphere (distance to closest surface point)
         * - Negative distance: point is inside the sphere (depth into the sphere)
         * - Zero: point is exactly on the sphere surface
         * 
         * The bounding sphere is computed via fitBoundingSphere() which is called automatically
         * after learnWorkspace() or loadWorkspaceFromFile().
         * 
         * @param point 3D point in workspace coordinates
         * @return Signed distance in meters (positive=outside, negative=inside)
         */
        double getDistance(const Eigen::Vector3d &point) const;

    private:
        /**
         * @brief Convert a pose to a 64-bit voxel key for hashing
         * 
         * Discretizes the 3D position into voxel coordinates and packs them into
         * a single 64-bit key (21 bits per axis). Supports ~±1000m range.
         * 
         * @param pose Input pose (only position is used)
         * @return 64-bit voxel key for hash set storage
         */
        uint64_t poseToVoxelKey(const geometry_msgs::msg::Pose &pose) const;

        /**
         * @brief Convert a voxel key back to a pose at the voxel center
         * @param key 64-bit voxel key
         * @return Pose with position at voxel center and identity orientation
         */
        geometry_msgs::msg::Pose voxelKeyToPose(uint64_t key) const;

        /**
         * @brief Discretize a continuous coordinate to voxel grid coordinate
         * @param coord Continuous coordinate in meters
         * @return Integer voxel coordinate
         */
        int32_t discretizeCoordinate(double coord) const;

        /**
         * @brief Fit a bounding sphere around the reachable workspace
         * 
         * Computes a minimal bounding sphere by:
         * 1. Computing the centroid of all reachable voxels
         * 2. Finding the furthest voxel from the centroid
         * 3. Setting sphere center to centroid and radius to furthest distance
         * 
         * This provides a conservative bound that contains all reachable positions.
         * Called automatically after learning or loading workspace.
         */
        void fitBoundingSphere();

        std::shared_ptr<MoveItInterface> moveit_interface_;
        std::shared_ptr<NBVVisualizer> visualizer_;
        std::unordered_set<uint64_t> reachable_voxels_;
        double voxel_size_ = 0.04; // Default voxel size in meters (this is seperate from the octomap voxel size)
        
        // Bounding sphere parameters
        Eigen::Vector3d sphere_center_ = Eigen::Vector3d::Zero();
        double sphere_radius_ = 0.0;
    };

}