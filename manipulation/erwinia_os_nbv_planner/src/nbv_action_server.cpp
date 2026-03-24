/**
 * @file nbv_action_server.cpp
 * @brief ROS 2 action server for running a single NBV baseline or volumetric scan.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <optional>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <stdexcept>
#include <cctype>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "erwinia_os_nbv_planner/moveit_interface.hpp"
#include "erwinia_os_nbv_planner/manipulation_workspace.hpp"
#include "erwinia_os_nbv_planner/octomap_interface.hpp"
#include "erwinia_os_nbv_planner/nbv_visualizer.hpp"
#include "erwinia_os_nbv_planner/viewpoint_generation.hpp"
#include "erwinia_os_nbv_planner/geometry_utils.hpp"
#include "erwinia_os_nbv_planner/conversions.hpp"
#include "erwinia_os_nbv_planner/nbv_planner_utils.hpp"

// This is generated from the RunNBV.action file by the ROS2 build system
#include "erwinia_os_nbv_planner/action/run_nbv.hpp"

using namespace std::chrono_literals;

using namespace erwinia_os_nbv_planner;
using namespace erwinia_os_nbv_planner::conversions;

class NBVActionServer : public rclcpp::Node
{
public:
    using RunNBV = erwinia_os_nbv_planner::action::RunNBV;
    using GoalHandleRunNBV = rclcpp_action::ServerGoalHandle<RunNBV>;

    NBVActionServer()
        : Node("nbv_action_server",
               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
    }

    void initialize()
    {
        auto node = shared_from_this();

        if (this->get_parameter("use_sim_time").as_bool())
        {
            waitForSimClock(node);
        }

        config_ = loadConfiguration(node);
        printConfiguration(config_, this->get_logger());

        // Load preset joint configurations (flat list of degrees, chunked by number of joints)
        if (this->has_parameter("preset_joint_configs_deg"))
        {
            std::vector<double> flat = this->get_parameter("preset_joint_configs_deg").as_double_array();
            const size_t n_joints = config_.init_joint_angles_rad.size();
            if (!flat.empty() && n_joints > 0 && flat.size() % n_joints == 0)
            {
                for (size_t i = 0; i < flat.size(); i += n_joints)
                {
                    std::vector<double> cfg(flat.begin() + i, flat.begin() + i + n_joints);
                    for (auto &v : cfg) v *= M_PI / 180.0;
                    preset_joint_configs_template_.push_back(cfg);
                }
                RCLCPP_INFO(this->get_logger(), "Loaded %zu preset joint configurations",
                            preset_joint_configs_template_.size());
            }
            else if (!flat.empty())
            {
                RCLCPP_WARN(this->get_logger(),
                            "preset_joint_configs_deg size (%zu) not divisible by n_joints (%zu), ignoring",
                            flat.size(), n_joints);
            }
        }

        clear_client_ = this->create_client<std_srvs::srv::Trigger>("/occupancy_map/clear");

        trigger_clients_ = createTriggerClients(node);
        stopVideoCapture(node, trigger_clients_, this->get_logger());

        moveit_interface_ = setupMoveItInterface(node, config_);
        RCLCPP_INFO(this->get_logger(),
                    "MoveIt interface initialized for group '%s' with base frame '%s'",
                    config_.manipulator_group_name.c_str(),
                    moveit_interface_->getPoseReferenceFrame().c_str());

        if (config_.visualize)
        {
            visualizer_ = std::make_shared<NBVVisualizer>(
                node, config_.map_frame, config_.visualization_topic);
        }

        manip_workspace_ = setupWorkspace(moveit_interface_, visualizer_, config_, this->get_logger());
        if (!manip_workspace_)
        {
            throw std::runtime_error("Failed to initialize manipulation workspace");
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        octomap_interface_ = std::make_shared<OctoMapInterface>(node, config_.octomap_topic, true);

        if (config_.capture_type == "continuous")
        {
            startContinuousCapture(node, trigger_clients_, this->get_logger());
        }
        else if (config_.capture_type == "triggered")
        {
            if (!trigger_clients_.send_trigger->wait_for_service(std::chrono::seconds(5)))
            {
                RCLCPP_WARN(this->get_logger(),
                            "send_trigger service not available at startup, continuing anyway");
            }
        }

        action_server_ = rclcpp_action::create_server<RunNBV>(
            this,
            "run_nbv",
            std::bind(&NBVActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NBVActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&NBVActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "NBV action server ready on action 'run_nbv'");
    }

private:
    struct RunPreparation
    {
        Eigen::Vector3d init_cam_position;
        std::array<double, 4> init_cam_orientation{};
        Eigen::Isometry3d moveit_to_octomap_eigen{Eigen::Isometry3d::Identity()};
    };

    struct RunSummary
    {
        int total_viewpoints_visited = 0;
        int final_cluster_count = 0;
        double final_bbox_coverage = 0.0;
        bool success = true;
        std::string message = "Completed successfully";
    };

    // -------------------------------------------------------------------------
    // Action callbacks
    // -------------------------------------------------------------------------

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const RunNBV::Goal> goal)
    {
        std::lock_guard<std::mutex> lock(execution_mutex_);

        const std::string planner = toLower(goal->planner_type);

        RCLCPP_INFO(this->get_logger(),
                    "Received RunNBV goal with planner_type='%s'",
                    goal->planner_type.c_str());

        if (busy_)
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal because server is already busy");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (planner != "baseline" && planner != "volumetric" && planner != "semantic")
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejecting goal: planner_type must be 'baseline', 'volumetric', or 'semantic'");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRunNBV>)
    {
        RCLCPP_WARN(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleRunNBV> goal_handle)
    {
        std::thread(
            [this, goal_handle]()
            {
                this->execute(goal_handle);
            })
            .detach();
    }

    // -------------------------------------------------------------------------
    // Main execution
    // -------------------------------------------------------------------------

    void execute(const std::shared_ptr<GoalHandleRunNBV> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            busy_ = true;
        }

        auto result = std::make_shared<RunNBV::Result>();

        try
        {
            const auto goal = goal_handle->get_goal();
            const std::string planner = toLower(goal->planner_type);

            publishFeedback(goal_handle, -1, getCurrentClusterCount(), getCurrentBBoxCoverage(), "preparing");

            RunPreparation prep = prepareRun(goal_handle);

            if (goal_handle->is_canceling())
            {
                finishCanceled(goal_handle, "Canceled during preparation");
                return;
            }

            RunSummary summary;
            if (planner == "baseline")
            {
                summary = runBaseline(goal_handle, prep);
            }
            else if (planner == "volumetric")
            {
                summary = runVolumetric(goal_handle, prep);
            }
            else if (planner == "semantic")
            {
                summary = runSemantic(goal_handle, prep);
            } 
            else 
            {
                throw std::runtime_error("Invalid planner type: " + planner);
            }

            if (goal_handle->is_canceling())
            {
                finishCanceled(goal_handle, "Canceled during execution");
                return;
            }

            finalizeRun(goal_handle, summary);

            result->success = summary.success;
            result->message = summary.message;
            result->total_viewpoints = summary.total_viewpoints_visited;
            result->final_cluster_count = summary.final_cluster_count;
            result->final_bbox_coverage = summary.final_bbox_coverage;

            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(),
                        "RunNBV succeeded: viewpoints=%d, final clusters=%d, final coverage=%.4f",
                        result->total_viewpoints,
                        result->final_cluster_count,
                        result->final_bbox_coverage);
        }
        catch (const std::exception &e)
        {
            result->success = false;
            result->message = e.what();
            result->total_viewpoints = 0;
            result->final_cluster_count = getCurrentClusterCount();
            result->final_bbox_coverage = getCurrentBBoxCoverage();

            safeCleanup();

            RCLCPP_ERROR(this->get_logger(), "RunNBV aborted: %s", e.what());
            goal_handle->abort(result);
        }

        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            busy_ = false;
        }
    }

    // -------------------------------------------------------------------------
    // Common run preparation / cleanup
    // -------------------------------------------------------------------------

    RunPreparation prepareRun(const std::shared_ptr<GoalHandleRunNBV> &goal_handle)
    {
        auto node = shared_from_this();
        RunPreparation prep;

        // Wait for the clear service to become available
        while (!clear_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                throw std::runtime_error("Interrupted while waiting for /occupancy_map/clear service");
            }
            if (goal_handle->is_canceling())
            {
                finishCanceled(goal_handle, "Canceled while waiting for /occupancy_map/clear service");
                throw std::runtime_error("Canceled");
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /occupancy_map/clear service...");
        }

        if (!callClearMap(node, clear_client_, this->get_logger()))
        {
            throw std::runtime_error("Failed to clear occupancy map");
        }

        if (visualizer_)
        {
            visualizer_->clearAllMarkers();
        }

        RCLCPP_INFO(this->get_logger(), "Moving to initial joint configuration...");
        if (!moveit_interface_->planToJointStateWithRetries(config_.init_joint_angles_rad))
        {
            throw std::runtime_error("Failed to move to initial joint configuration");
        }

        if (!moveit_interface_->getLinkPose(
                config_.camera_optical_link,
                prep.init_cam_position,
                prep.init_cam_orientation))
        {
            throw std::runtime_error("Failed to get initial camera pose");
        }

        geometry_msgs::msg::Pose init_ee_pose;
        if (!moveit_interface_->getCurrentEndEffectorPose(init_ee_pose))
        {
            throw std::runtime_error("Failed to get current end-effector pose");
        }

        moveit_interface_->setOrientationConstraints(
            moveit_interface_->getEndEffectorLink(),
            init_ee_pose.orientation,
            M_PI / 2, M_PI / 2, M_PI / 2);

        waitForOctomap(node, octomap_interface_, trigger_clients_, config_, this->get_logger());

        geometry_msgs::msg::TransformStamped moveit_to_octomap_transform;
        try
        {
            moveit_to_octomap_transform = getMoveitToOctomapTransform(
                moveit_interface_, octomap_interface_, tf_buffer_, this->get_logger());
        }
        catch (const tf2::TransformException &e)
        {
            throw std::runtime_error(std::string("Failed to get MoveIt->OctoMap transform: ") + e.what());
        }

        prep.moveit_to_octomap_eigen = tf2::transformToEigen(moveit_to_octomap_transform.transform);

        if (!geometry_utils::isIdentityTransform(prep.moveit_to_octomap_eigen))
        {
            throw std::runtime_error(
                "MoveIt and OctoMap frames are not aligned; this demo expects them to be the same");
        }

        publishFeedback(goal_handle, -1, getCurrentClusterCount(), getCurrentBBoxCoverage(), "ready");
        return prep;
    }

    void finalizeRun(
        const std::shared_ptr<GoalHandleRunNBV> &goal_handle,
        RunSummary &summary)
    {
        publishFeedback(
            goal_handle,
            summary.total_viewpoints_visited,
            getCurrentClusterCount(),
            getCurrentBBoxCoverage(),
            "returning_home");

        RCLCPP_INFO(this->get_logger(), "Returning to initial joint configuration...");
        if (!moveit_interface_->planToJointStateWithRetries(config_.init_joint_angles_rad))
        {
            throw std::runtime_error("Failed to return to initial joint configuration");
        }

        summary.final_cluster_count = getCurrentClusterCount();
        summary.final_bbox_coverage = getCurrentBBoxCoverage();

        if (visualizer_)
        {
            visualizer_->clearAllMarkers();
        }

        moveit_interface_->clearPathConstraints();
    }

    void safeCleanup()
    {
        if (visualizer_)
        {
            visualizer_->clearAllMarkers();
        }
        if (moveit_interface_)
        {
            moveit_interface_->clearPathConstraints();
        }
    }

    void finishCanceled(
        const std::shared_ptr<GoalHandleRunNBV> &goal_handle,
        const std::string &message)
    {
        auto result = std::make_shared<RunNBV::Result>();
        result->success = false;
        result->message = message;
        result->total_viewpoints = 0;
        result->final_cluster_count = getCurrentClusterCount();
        result->final_bbox_coverage = getCurrentBBoxCoverage();

        safeCleanup();
        goal_handle->canceled(result);

        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            busy_ = false;
        }

        RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    }

    // -------------------------------------------------------------------------
    // Shared planner helpers
    // -------------------------------------------------------------------------

    bool checkCanceledOrShutdown(const std::shared_ptr<GoalHandleRunNBV> &goal_handle) const
    {
        return goal_handle->is_canceling() || !rclcpp::ok();
    }

    void publishRunFeedback(
        const std::shared_ptr<GoalHandleRunNBV> &goal_handle,
        int current_viewpoint,
        const std::string &state)
    {
        publishFeedback(
            goal_handle,
            current_viewpoint,
            getCurrentClusterCount(),
            getCurrentBBoxCoverage(),
            state);
    }

    std::vector<Viewpoint> generatePlaneReachableViewpoints(const RunPreparation &prep)
    {
        Eigen::Vector3d cam_pos_map = prep.moveit_to_octomap_eigen * prep.init_cam_position;
        auto [plane_corners_map, distance] = computePlane(octomap_interface_, cam_pos_map);

        if (visualizer_)
        {
            std_msgs::msg::ColorRGBA plane_color;
            plane_color.r = 0.0f;
            plane_color.g = 1.0f;
            plane_color.b = 0.0f;
            plane_color.a = 0.5f;
            visualizer_->publishPlane(plane_corners_map, "nbv_plane", 0.02, plane_color);
        }

        Eigen::Quaterniond init_cam_quat_map =
            Eigen::Quaterniond(prep.moveit_to_octomap_eigen.rotation()) *
            geometry_utils::arrayToEigenQuat(prep.init_cam_orientation);

        auto [all_viewpoints_map, coverage_planes_map] = generateViewpointsFromPlane(
            plane_corners_map,
            distance,
            init_cam_quat_map,
            config_.viewpoint_overlap_ratio,
            config_.camera_horizontal_fov_rad,
            config_.camera_vertical_fov_rad);

        (void)coverage_planes_map;

        auto reachable_viewpoints = filterReachableViewpoints(
            all_viewpoints_map,
            manip_workspace_,
            moveit_interface_,
            config_,
            this->get_logger());

        return reachable_viewpoints;
    }

    void visualizeViewpoints(const std::vector<Viewpoint> &viewpoints)
    {
        if (!visualizer_)
        {
            return;
        }

        std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
        viewpoint_poses.reserve(viewpoints.size());

        for (const auto &vp : viewpoints)
        {
            viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
        }

        visualizer_->publishCoordinates(
            viewpoint_poses,
            0.15,
            0.01,
            0.5f,
            "reachable_viewpoints",
            moveit_interface_->getPoseReferenceFrame());
    }

    bool executeViewpointMotion(const Viewpoint &viewpoint)
    {
        auto plan = planPathsToViewpoint(
            viewpoint,
            moveit_interface_,
            config_,
            this->get_logger(),
            config_.hint_joint_configs);

        if (!plan)
        {
            return false;
        }

        return executeAndWaitForMotion(moveit_interface_, shared_from_this(), *plan, this->get_logger());
    }

    // -------------------------------------------------------------------------
    // Baseline planner
    // -------------------------------------------------------------------------

    RunSummary runBaseline(
        const std::shared_ptr<GoalHandleRunNBV> &goal_handle,
        const RunPreparation &prep)
    {
        RunSummary summary;
        RCLCPP_INFO(this->get_logger(), "Running baseline planner");

        // Execute preset joint configurations before planned viewpoints
        std::deque<std::vector<double>> preset_configs(
            preset_joint_configs_template_.begin(), preset_joint_configs_template_.end());

        for (size_t pi = 0; !preset_configs.empty(); ++pi)
        {
            if (checkCanceledOrShutdown(goal_handle))
            {
                summary.success = false;
                summary.message = "Baseline run interrupted during presets";
                return summary;
            }

            RCLCPP_INFO(this->get_logger(), "Executing preset joint configuration %zu", pi);
            auto joint_config = preset_configs.front();
            preset_configs.pop_front();

            if (!moveit_interface_->planToJointStateWithRetries(joint_config))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to execute preset config %zu, skipping", pi);
                continue;
            }

            waitForOctomap(shared_from_this(), octomap_interface_, trigger_clients_, config_, this->get_logger());
            publishRunFeedback(goal_handle, static_cast<int>(pi), "baseline_preset");

            if (visualizer_)
                visualizer_->clearAllMarkers();
        }

        auto reachable_viewpoints = generatePlaneReachableViewpoints(prep);
        if (reachable_viewpoints.empty())
        {
            throw std::runtime_error("No reachable viewpoints found for baseline planner");
        }

        lawnmowerSortViewpoints(reachable_viewpoints);
        visualizeViewpoints(reachable_viewpoints);

        for (size_t i = 0; i < reachable_viewpoints.size(); ++i)
        {
            if (checkCanceledOrShutdown(goal_handle))
            {
                summary.success = false;
                summary.message = "Baseline run interrupted";
                return summary;
            }

            RCLCPP_INFO(this->get_logger(),
                        "Baseline viewpoint %d",
                        summary.total_viewpoints_visited);

            auto plan = planPathsToViewpoint(
                reachable_viewpoints[i],
                moveit_interface_,
                config_,
                this->get_logger(),
                config_.hint_joint_configs);

            if (!plan)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to plan to viewpoint %zu, skipping",
                            i);
                continue;
            }

            if (!executeAndWaitForMotion(moveit_interface_, shared_from_this(), *plan, this->get_logger()))
            {
                throw std::runtime_error("Motion execution failed during baseline planner");
            }

            waitForOctomap(shared_from_this(), octomap_interface_, trigger_clients_, config_, this->get_logger());

            summary.total_viewpoints_visited++;

            publishRunFeedback(goal_handle, static_cast<int>(i), "baseline_running");

            if (visualizer_)
            {
                visualizer_->clearAllMarkers();
            }
        }

        summary.final_cluster_count = getCurrentClusterCount();
        summary.final_bbox_coverage = getCurrentBBoxCoverage();
        summary.success = true;
        summary.message = "Baseline planner completed successfully";
        return summary;
    }

    // -------------------------------------------------------------------------
    // Volumetric planner
    // -------------------------------------------------------------------------

    RunSummary runVolumetric(
        const std::shared_ptr<GoalHandleRunNBV> &goal_handle,
        const RunPreparation &prep)
    {
        RunSummary summary;
        RCLCPP_INFO(this->get_logger(), "Running volumetric planner");

        // Execute preset joint configurations before NBV iterations
        std::deque<std::vector<double>> preset_configs(
            preset_joint_configs_template_.begin(), preset_joint_configs_template_.end());

        for (size_t pi = 0; !preset_configs.empty(); ++pi)
        {
            if (checkCanceledOrShutdown(goal_handle))
            {
                summary.success = false;
                summary.message = "Volumetric run interrupted during presets";
                return summary;
            }

            RCLCPP_INFO(this->get_logger(), "Executing preset joint configuration %zu", pi);
            auto joint_config = preset_configs.front();
            preset_configs.pop_front();

            if (!moveit_interface_->planToJointStateWithRetries(joint_config))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to execute preset config %zu, skipping", pi);
                continue;
            }

            waitForOctomap(shared_from_this(), octomap_interface_, trigger_clients_, config_, this->get_logger());
            publishRunFeedback(goal_handle, static_cast<int>(pi), "volumetric_preset");

            if (visualizer_)
                visualizer_->clearAllMarkers();
        }

        for (int i = 0; i < config_.max_iterations; ++i)
        {
            if (checkCanceledOrShutdown(goal_handle))
            {
                summary.success = false;
                summary.message = "Volumetric run interrupted";
                return summary;
            }

            RCLCPP_INFO(this->get_logger(),
                        "Volumetric viewpoint %d",
                        summary.total_viewpoints_visited);

            std::vector<octomap::point3d> frontiers = octomap_interface_->findFrontiers(
                config_.min_unknown_neighbors,
                octomap_interface_->hasBoundingBox());

            if (frontiers.empty())
            {
                RCLCPP_WARN(this->get_logger(),
                            "No frontiers found, exploration may be complete");
                break;
            }

            std::vector<Eigen::Vector3d> frontiers_eigen = octomapVectorToEigen(frontiers);

            std::vector<Eigen::Vector3d> viewable_frontiers;
            for (const auto &f : frontiers_eigen)
            {
                if (manip_workspace_->getDistance(f) < config_.ideal_camera_distance / 2.0)
                {
                    viewable_frontiers.push_back(f);
                }
            }

            if (viewable_frontiers.empty())
            {
                RCLCPP_WARN(this->get_logger(),
                            "No viewable frontiers found within workspace");
            }

            int n_clusters = std::max(1, static_cast<int>(viewable_frontiers.size()) / 100);
            std::vector<Cluster> frontier_clusters = octomap_interface_->kmeansCluster(
                eigenVectorToOctomap(viewable_frontiers), n_clusters, 50, 1e-4);

            auto plane_viewpoints = generatePlaneReachableViewpoints(prep);

            auto spherical_cap_viewpoints = generateSphericalCaps(
                plane_viewpoints,
                prep.init_cam_orientation,
                config_.cap_max_theta_rad,
                config_.cap_min_theta_rad);

            std::vector<Eigen::Vector3d> cluster_centers;
            for (const auto &cluster : frontier_clusters)
            {
                cluster_centers.push_back(octomapToEigen(cluster.center));
            }

            const double min_distance =
                std::max(0.0, config_.ideal_camera_distance - config_.ideal_distance_tolerance);
            const double max_distance =
                config_.ideal_camera_distance + config_.ideal_distance_tolerance;

            std::vector<Viewpoint> frontier_viewpoints = generateFrontierBasedViewpoints(
                cluster_centers,
                prep.init_cam_orientation,
                min_distance,
                max_distance,
                config_.num_viewpoints_per_frontier,
                false,
                config_.z_bias_sigma,
                0.05,
                1000,
                this->get_logger());

            std::vector<Viewpoint> total_viewpoints;
            total_viewpoints.insert(total_viewpoints.end(),
                                    spherical_cap_viewpoints.begin(),
                                    spherical_cap_viewpoints.end());
            total_viewpoints.insert(total_viewpoints.end(),
                                    frontier_viewpoints.begin(),
                                    frontier_viewpoints.end());

            if (total_viewpoints.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No viewpoints generated");
                break;
            }

            auto reachable_viewpoints = filterReachableViewpoints(
                total_viewpoints,
                manip_workspace_,
                moveit_interface_,
                config_,
                this->get_logger());

            if (reachable_viewpoints.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No reachable viewpoints found");
                break;
            }

            visualizeViewpoints(reachable_viewpoints);

            Eigen::Vector3d current_cam_position;
            std::array<double, 4> current_cam_orientation{};
            moveit_interface_->getLinkPose(
                config_.camera_optical_link,
                current_cam_position,
                current_cam_orientation);

            for (auto &vp : reachable_viewpoints)
            {
                vp.information_gain = computeInformationGain(
                    vp,
                    octomap_interface_,
                    config_.camera_horizontal_fov_rad,
                    config_.camera_vertical_fov_rad,
                    config_.camera_scaled_width,
                    config_.camera_scaled_height,
                    config_.camera_max_range,
                    octomap_interface_->getResolution(),
                    config_.num_camera_rays,
                    octomap_interface_->hasBoundingBox(),
                    this->get_logger());

                const double distance_cost = (vp.position - current_cam_position).norm();
                vp.cost = distance_cost;
                vp.utility = vp.information_gain - config_.alpha_cost_weight * distance_cost;
            }

            std::priority_queue<Viewpoint> viewpoint_heap;
            for (const auto &vp : reachable_viewpoints)
            {
                viewpoint_heap.push(vp);
            }

            std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_plan;
            Viewpoint best_viewpoint;

            while (!viewpoint_heap.empty())
            {
                best_viewpoint = viewpoint_heap.top();
                viewpoint_heap.pop();

                if (best_viewpoint.information_gain < config_.min_information_gain)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "No viewpoints above min information gain threshold %.4f",
                                config_.min_information_gain);
                    break;
                }

                best_plan = planPathsToViewpoint(
                    best_viewpoint,
                    moveit_interface_,
                    config_,
                    this->get_logger(),
                    config_.hint_joint_configs);

                if (!best_plan)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Failed to plan to best viewpoint, trying next one...");
                    continue;
                }

                break;
            }

            if (!best_plan)
            {
                RCLCPP_WARN(this->get_logger(), "No valid viewpoint with feasible plan found");
                break;
            }

            if (!executeAndWaitForMotion(moveit_interface_, shared_from_this(), *best_plan, this->get_logger()))
            {
                throw std::runtime_error("Motion execution failed during volumetric planner");
            }

            waitForOctomap(shared_from_this(), octomap_interface_, trigger_clients_, config_, this->get_logger());

            summary.total_viewpoints_visited++;

            publishRunFeedback(goal_handle, i, "volumetric_running");

            if (visualizer_)
            {
                visualizer_->clearAllMarkers();
            }
        }

        summary.final_cluster_count = getCurrentClusterCount();
        summary.final_bbox_coverage = getCurrentBBoxCoverage();
        summary.success = true;
        summary.message = "Volumetric planner completed successfully";
        return summary;
    }

    // -------------------------------------------------------------------------
    // Semantic planner
    // -------------------------------------------------------------------------

    RunSummary runSemantic(
        const std::shared_ptr<GoalHandleRunNBV> &goal_handle,
        const RunPreparation &prep)
    {
        RunSummary summary;
        RCLCPP_INFO(this->get_logger(), "Running semantic planner");

        // Execute preset joint configurations before NBV iterations
        std::deque<std::vector<double>> preset_configs(
            preset_joint_configs_template_.begin(), preset_joint_configs_template_.end());

        for (size_t pi = 0; !preset_configs.empty(); ++pi)
        {
            if (checkCanceledOrShutdown(goal_handle))
            {
                summary.success = false;
                summary.message = "Semantic run interrupted during presets";
                return summary;
            }

            RCLCPP_INFO(this->get_logger(), "Executing preset joint configuration %zu", pi);
            auto joint_config = preset_configs.front();
            preset_configs.pop_front();

            if (!moveit_interface_->planToJointStateWithRetries(joint_config))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to execute preset config %zu, skipping", pi);
                continue;
            }

            waitForOctomap(shared_from_this(), octomap_interface_, trigger_clients_, config_, this->get_logger());
            publishRunFeedback(goal_handle, static_cast<int>(pi), "semantic_preset");

            if (visualizer_)
                visualizer_->clearAllMarkers();
        }

        for (int i = 0; i < config_.max_iterations; ++i)
        {
            if (checkCanceledOrShutdown(goal_handle))
            {
                summary.success = false;
                summary.message = "Semantic run interrupted";
                return summary;
            }

            RCLCPP_INFO(this->get_logger(),
                        "Semantic viewpoint %d",
                        summary.total_viewpoints_visited);

            // Find voxels whose confidence is below max_semantic_certainty
            std::vector<octomap::point3d> frontiers = octomap_interface_->getUncertainVoxels(
                config_.max_semantic_certainty, true, true);
            // NOTE: no break on empty — semantic planner relies on max_iterations to terminate

            std::vector<Eigen::Vector3d> frontiers_eigen = octomapVectorToEigen(frontiers);

            std::vector<Eigen::Vector3d> viewable_frontiers;
            for (const auto &f : frontiers_eigen)
            {
                if (manip_workspace_->getDistance(f) < config_.ideal_camera_distance / 2.0)
                    viewable_frontiers.push_back(f);
            }

            if (viewable_frontiers.empty())
                RCLCPP_WARN(this->get_logger(), "No viewable uncertain voxels within workspace");

            int n_clusters = std::max(1, static_cast<int>(viewable_frontiers.size()) / 100);
            std::vector<Cluster> frontier_clusters = octomap_interface_->kmeansCluster(
                eigenVectorToOctomap(viewable_frontiers), n_clusters, 50, 1e-4);

            auto plane_viewpoints = generatePlaneReachableViewpoints(prep);

            auto spherical_cap_viewpoints = generateSphericalCaps(
                plane_viewpoints,
                prep.init_cam_orientation,
                config_.cap_max_theta_rad,
                config_.cap_min_theta_rad);

            std::vector<Eigen::Vector3d> cluster_centers;
            for (const auto &cluster : frontier_clusters)
                cluster_centers.push_back(octomapToEigen(cluster.center));

            const double min_distance =
                std::max(0.0, config_.ideal_camera_distance - config_.ideal_distance_tolerance);
            const double max_distance =
                config_.ideal_camera_distance + config_.ideal_distance_tolerance;

            std::vector<Viewpoint> frontier_viewpoints = generateFrontierBasedViewpoints(
                cluster_centers,
                prep.init_cam_orientation,
                min_distance,
                max_distance,
                config_.num_viewpoints_per_frontier,
                false,
                config_.z_bias_sigma,
                0.05,
                1000,
                this->get_logger());

            std::vector<Viewpoint> total_viewpoints;
            total_viewpoints.insert(total_viewpoints.end(),
                                    spherical_cap_viewpoints.begin(),
                                    spherical_cap_viewpoints.end());
            total_viewpoints.insert(total_viewpoints.end(),
                                    frontier_viewpoints.begin(),
                                    frontier_viewpoints.end());

            if (total_viewpoints.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No viewpoints generated");
                break;
            }

            auto reachable_viewpoints = filterReachableViewpoints(
                total_viewpoints,
                manip_workspace_,
                moveit_interface_,
                config_,
                this->get_logger());

            if (reachable_viewpoints.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No reachable viewpoints found");
                break;
            }

            visualizeViewpoints(reachable_viewpoints);

            Eigen::Vector3d current_cam_position;
            std::array<double, 4> current_cam_orientation{};
            moveit_interface_->getLinkPose(
                config_.camera_optical_link,
                current_cam_position,
                current_cam_orientation);

            for (auto &vp : reachable_viewpoints)
            {
                vp.information_gain = computeSemanticInformationGain(
                    vp,
                    octomap_interface_,
                    config_.camera_horizontal_fov_rad,
                    config_.camera_vertical_fov_rad,
                    config_.camera_scaled_width,
                    config_.camera_scaled_height,
                    config_.camera_max_range,
                    octomap_interface_->getResolution(),
                    config_.num_camera_rays,
                    octomap_interface_->hasBoundingBox(),
                    config_.beta_semantic_weight,
                    true,
                    this->get_logger());

                const double distance_cost = (vp.position - current_cam_position).norm();
                vp.cost = distance_cost;
                vp.utility = vp.information_gain - config_.alpha_cost_weight * distance_cost;
            }

            std::priority_queue<Viewpoint> viewpoint_heap;
            for (const auto &vp : reachable_viewpoints)
                viewpoint_heap.push(vp);

            std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_plan;
            Viewpoint best_viewpoint;

            while (!viewpoint_heap.empty())
            {
                best_viewpoint = viewpoint_heap.top();
                viewpoint_heap.pop();

                if (best_viewpoint.information_gain < config_.min_information_gain)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "No viewpoints above min information gain threshold %.4f",
                                config_.min_information_gain);
                    break;
                }

                best_plan = planPathsToViewpoint(
                    best_viewpoint,
                    moveit_interface_,
                    config_,
                    this->get_logger(),
                    config_.hint_joint_configs);

                if (!best_plan)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Failed to plan to best viewpoint, trying next one...");
                    continue;
                }

                break;
            }

            if (!best_plan)
            {
                RCLCPP_WARN(this->get_logger(), "No valid viewpoint with feasible plan found");
                break;
            }

            if (!executeAndWaitForMotion(moveit_interface_, shared_from_this(), *best_plan, this->get_logger()))
                throw std::runtime_error("Motion execution failed during semantic planner");

            waitForOctomap(shared_from_this(), octomap_interface_, trigger_clients_, config_, this->get_logger());

            summary.total_viewpoints_visited++;

            publishRunFeedback(goal_handle, i, "semantic_running");

            if (visualizer_)
                visualizer_->clearAllMarkers();
        }

        summary.final_cluster_count = getCurrentClusterCount();
        summary.final_bbox_coverage = getCurrentBBoxCoverage();
        summary.success = true;
        summary.message = "Semantic planner completed successfully";
        return summary;
    }

    // -------------------------------------------------------------------------
    // Feedback / state helpers
    // -------------------------------------------------------------------------

    void publishFeedback(
        const std::shared_ptr<GoalHandleRunNBV> &goal_handle,
        int32_t current_viewpoint,
        int32_t cluster_count,
        double bbox_coverage,
        const std::string &state)
    {
        auto feedback = std::make_shared<RunNBV::Feedback>();
        feedback->current_viewpoint = current_viewpoint;
        feedback->cluster_count = cluster_count;
        feedback->bbox_coverage = bbox_coverage;
        feedback->state = state;
        goal_handle->publish_feedback(feedback);
    }

    int getCurrentClusterCount() const
    {
        if (!octomap_interface_ || !octomap_interface_->isSemanticTree())
        {
            return 0;
        }

        auto clusters = octomap_interface_->clusterSemanticVoxels(false);
        return static_cast<int>(clusters.size());
    }

    double getCurrentBBoxCoverage() const
    {
        if (!octomap_interface_)
        {
            return 0.0;
        }

        return octomap_interface_->calculateCoverage();
    }

    static std::string toLower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

    NBVPlannerConfig config_{};
    std::vector<std::vector<double>> preset_joint_configs_template_;
    TriggerClients trigger_clients_{};

    rclcpp_action::Server<RunNBV>::SharedPtr action_server_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;

    std::shared_ptr<MoveItInterface> moveit_interface_;
    std::shared_ptr<ManipulationWorkspace> manip_workspace_;
    std::shared_ptr<OctoMapInterface> octomap_interface_;
    std::shared_ptr<NBVVisualizer> visualizer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex execution_mutex_;
    bool busy_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    try
    {
        auto node = std::make_shared<NBVActionServer>();
        node->initialize();
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to start NBVActionServer: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}