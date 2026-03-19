/**
 * @file nbv_baseline_demo.cpp
 * @brief Next-Best-View baseline demo for manipulation workspace
 */

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <queue>
#include <algorithm>
#include <optional>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <thread>

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

using namespace erwinia_os_nbv_planner;
using namespace erwinia_os_nbv_planner::conversions;

static std::string formatRunName(int idx)
{
    std::ostringstream oss;
    oss << "run_" << std::setw(3) << std::setfill('0') << idx;
    return oss.str();
}

static int run(std::shared_ptr<rclcpp::Node> node)
{
    int n_runs = node->get_parameter("n_runs").as_int();
    if (n_runs < 1) n_runs = 1;

    // Service client to clear the occupancy map between runs
    auto clear_client = node->create_client<std_srvs::srv::Trigger>("/occupancy_map/clear");

    // Wait for sim time if needed
    if (node->get_parameter("use_sim_time").as_bool())
        waitForSimClock(node);

    // Load configuration (we will mutate metrics dirs per run if n_runs > 1)
    auto config = loadConfiguration(node);
    printConfiguration(config, node->get_logger());

    // Load preset joint configurations (flat list of degrees, chunked by number of joints)
    std::vector<std::vector<double>> preset_joint_configs_template;
    if (node->has_parameter("preset_joint_configs_deg"))
    {
        std::vector<double> flat = node->get_parameter("preset_joint_configs_deg").as_double_array();
        const size_t n_joints = config.init_joint_angles_rad.size();
        if (!flat.empty() && n_joints > 0 && flat.size() % n_joints == 0)
        {
            for (size_t i = 0; i < flat.size(); i += n_joints)
            {
                std::vector<double> cfg(flat.begin() + i, flat.begin() + i + n_joints);
                for (auto &v : cfg) v *= M_PI / 180.0;
                preset_joint_configs_template.push_back(cfg);
            }
            RCLCPP_INFO(node->get_logger(), "Loaded %zu preset joint configurations", preset_joint_configs_template.size());
        }
        else if (!flat.empty())
        {
            RCLCPP_WARN(node->get_logger(), "preset_joint_configs_deg size (%zu) not divisible by n_joints (%zu), ignoring", flat.size(), n_joints);
        }
    }

    // Save base metrics dirs (when n_runs > 1 these are expected to be the base folder)
    const std::string base_metrics_plots_dir = config.metrics_plots_dir;
    const std::string base_metrics_data_dir  = config.metrics_data_dir;

    // Create trigger clients and stop any ongoing video capture
    auto trigger_clients = createTriggerClients(node);
    stopVideoCapture(node, trigger_clients, node->get_logger());

    // Initialize MoveIt interface
    auto moveit_interface = setupMoveItInterface(node, config);
    RCLCPP_DEBUG(node->get_logger(), "MoveIt Interface initialized for group: %s with base link: %s",
                config.manipulator_group_name.c_str(), moveit_interface->getPoseReferenceFrame().c_str());

    // Initialize visualizer if requested
    std::shared_ptr<NBVVisualizer> visualizer;
    if (config.visualize)
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);

    // Setup workspace with moveit interface (done once, reused)
    auto manip_workspace = setupWorkspace(moveit_interface, visualizer, config, node->get_logger());
    if (!manip_workspace)
    {
        return 1;
    }

    // Initialize TF2 buffer and listener
    RCLCPP_DEBUG(node->get_logger(), "\nInitializing TF2 Buffer and Listener");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize octomap interface (subscriber stays alive across runs)
    RCLCPP_DEBUG(node->get_logger(), "\nInitializing OctoMap Interface");
    auto octomap_interface = std::make_shared<OctoMapInterface>(node, config.octomap_topic, true);

    // Handle camera triggering based on capture_type (done once)
    if (config.capture_type == "continuous")
    {
        startContinuousCapture(node, trigger_clients, node->get_logger());
    }
    else if (config.capture_type == "triggered")
    {
        if (!trigger_clients.send_trigger->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(node->get_logger(), "send_trigger service not available, waiting for octomap anyway...");
    }

    // =========================
    // Multi-run outer loop
    // =========================
    for (int run_idx = 1; run_idx <= n_runs; ++run_idx)
    {
        RCLCPP_INFO(node->get_logger(), "\n==============================");
        RCLCPP_INFO(node->get_logger(), "NBV BASELINE RUN %d / %d", run_idx, n_runs);
        RCLCPP_INFO(node->get_logger(), "==============================\n");

        // Metrics dir behavior:
        // - n_runs == 1: keep config.metrics_* as passed in (likely .../plots and .../data)
        // - n_runs > 1 : create .../run_###/{plots,data}
        if (n_runs > 1)
        {
            const std::string run_name = formatRunName(run_idx);

            config.metrics_plots_dir =
                (std::filesystem::path(base_metrics_plots_dir) / run_name / "plots").string();
            config.metrics_data_dir =
                (std::filesystem::path(base_metrics_data_dir) / run_name / "data").string();

            std::filesystem::create_directories(config.metrics_plots_dir);
            std::filesystem::create_directories(config.metrics_data_dir);

            RCLCPP_INFO(node->get_logger(), "Run plots dir: %s", config.metrics_plots_dir.c_str());
            RCLCPP_INFO(node->get_logger(), "Run data dir : %s", config.metrics_data_dir.c_str());
        }
        else
        {
            config.metrics_plots_dir = base_metrics_plots_dir;
            config.metrics_data_dir  = base_metrics_data_dir;
        }

        // Clear occupancy map before starting run (includes run 1)
        if (!callClearMap(node, clear_client, node->get_logger()))
        {
            return 1;
        }

        // Clear previous visualization markers
        if (visualizer)
        {
            visualizer->clearAllMarkers();
        }

        // Reset preset joint configurations for this run
        std::deque<std::vector<double>> preset_configs(
            preset_joint_configs_template.begin(), preset_joint_configs_template.end());

        // Move to initial joint configuration
        RCLCPP_DEBUG(node->get_logger(), "\nMoving to initial joint configuration...");
        if (!moveit_interface->planToJointStateWithRetries(config.init_joint_angles_rad))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
            return 1;
        }

        // Get initial camera pose wrt moveit base link (per run, in case robot moved)
        Eigen::Vector3d init_cam_position;
        std::array<double, 4> init_cam_orientation;
        if (!moveit_interface->getLinkPose(config.camera_optical_link, init_cam_position, init_cam_orientation))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get camera link pose");
            return 1;
        }

        // Set orientation constraints (±90 degrees tolerance)
        // Read current EE pose directly — arm is already at init config, no FK race condition
        geometry_msgs::msg::Pose init_ee_pose;
        if (moveit_interface->getCurrentEndEffectorPose(init_ee_pose))
        {
            moveit_interface->setOrientationConstraints(moveit_interface->getEndEffectorLink(),
                init_ee_pose.orientation, M_PI/2, M_PI/2, M_PI/2);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get current end-effector pose for orientation constraints");
            return 1;
        }

        // // Keep EE in front of robot (Y ≥ 0), all other axes free
        // moveit_interface->setPositionConstraints(moveit_interface->getEndEffectorLink(),
        //     MoveItInterface::UNCONSTRAINED, MoveItInterface::UNCONSTRAINED,   // x
        //     MoveItInterface::UNCONSTRAINED, 0.0,   // y
        //     MoveItInterface::UNCONSTRAINED, MoveItInterface::UNCONSTRAINED);  // z

        // Wait for initial octomap (fresh after clear)
        waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

        // Initialize evaluation if enabled (per run)
        std::vector<EvaluationMetrics> all_metrics;
        double initial_time = 0.0;

        if (config.enable_evaluation && octomap_interface->isSemanticTree())
        {
            if (!octomap_interface->loadGroundTruthSemantics(config.gt_points_file))
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to load ground truth file: %s", config.gt_points_file.c_str());
                return 1;
            }
            RCLCPP_INFO(node->get_logger(), "Ground truth loaded successfully");

            // Store initial time for relative time calculations
            initial_time = node->now().seconds();

            // Perform initial evaluation
            EvaluationMetrics eval_metrics;
            eval_metrics.time = 0.0;
            auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
            auto match_result = octomap_interface->matchClustersToGroundTruth(
                latest_clusters, config.eval_threshold_radius, false);
            eval_metrics.class_metrics = octomap_interface->evaluateMatchResults(match_result, false);

            auto [occupied, free] = octomap_interface->getVoxelCounts();
            eval_metrics.occupied_voxels = occupied;
            eval_metrics.free_voxels = free;

            eval_metrics.bbox_coverage = octomap_interface->calculateCoverage();
            all_metrics.push_back(eval_metrics);

            if (visualizer)
            {
                visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
            }
            // visualizer->plotAllMetrics(all_metrics, config.metrics_plots_dir);
            visualizer->logAllMetricsToCSV(all_metrics, config.metrics_data_dir);
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Evaluation disabled or octomap is not semantic, skipping evaluation setup");
        }

        // Get transform between MoveIt and OctoMap frames
        geometry_msgs::msg::TransformStamped moveit_to_octomap_transform;
        try
        {
            moveit_to_octomap_transform = getMoveitToOctomapTransform(
                moveit_interface, octomap_interface, tf_buffer, node->get_logger());
        }
        catch (const tf2::TransformException &)
        {
            return 1;
        }

        Eigen::Isometry3d transform_eigen = tf2::transformToEigen(moveit_to_octomap_transform.transform);

        // Check if the transform is identity (we must have aligned frames for this demo)
        if (!geometry_utils::isIdentityTransform(transform_eigen))
        {
            RCLCPP_ERROR(node->get_logger(), "MoveIt and OctoMap frames are not aligned, they must be the same for this demo");
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "\n=== Generate NBV Viewpoints ===");
        Eigen::Vector3d cam_pos_map = transform_eigen * init_cam_position;
        auto [plane_corners_map, distance] = computePlane(octomap_interface, cam_pos_map);

        if (visualizer)
        {
            RCLCPP_DEBUG(node->get_logger(), "Publishing NBV midplane for visualization");
            std_msgs::msg::ColorRGBA plane_color;
            plane_color.r = 0.0f;
            plane_color.g = 1.0f;
            plane_color.b = 0.0f;
            plane_color.a = 0.5f;
            visualizer->publishPlane(plane_corners_map, "nbv_plane", 0.02, plane_color);
        }

        // Generate viewpoints on the plane
        Eigen::Quaterniond init_cam_quat_map =
            Eigen::Quaterniond(transform_eigen.rotation()) * geometry_utils::arrayToEigenQuat(init_cam_orientation);

        auto [all_viewpoints_map, coverage_planes_map] = generateViewpointsFromPlane(
            plane_corners_map, distance, init_cam_quat_map,
            config.viewpoint_overlap_ratio, config.camera_horizontal_fov_rad, config.camera_vertical_fov_rad);

        // if (visualizer)
        // {
        //     RCLCPP_DEBUG(node->get_logger(), "Publishing coverage planes for visualization");
        //     for (size_t i = 0; i < coverage_planes_map.size(); ++i)
        //     {
        //         std_msgs::msg::ColorRGBA coverage_color;
        //         coverage_color.r = 0.0f;
        //         coverage_color.g = 0.5f;
        //         coverage_color.b = 1.0f;
        //         coverage_color.a = 0.3f;
        //         visualizer->publishPlane(coverage_planes_map[i], "coverage_" + std::to_string(i), 0.01, coverage_color);
        //         std::this_thread::sleep_for(std::chrono::milliseconds(50));
        //     }
        // }

        auto all_viewpoints = all_viewpoints_map; // Since frames are aligned

        RCLCPP_INFO(node->get_logger(), "\n=== Filter Reachable Viewpoints ===");
        auto reachable_viewpoints = filterReachableViewpoints(all_viewpoints, manip_workspace, moveit_interface, config, node->get_logger());
        if (reachable_viewpoints.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "No reachable viewpoints found");
            return 1;
        }

        // Sort reachable viewpoints
        lawnmowerSortViewpoints(reachable_viewpoints);

        if (visualizer)
        {
            std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
            viewpoint_poses.reserve(reachable_viewpoints.size());
            for (const auto &vp : reachable_viewpoints)
            {
                viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
            }
            visualizer->publishCoordinates(
                viewpoint_poses, 0.15, 0.01, 0.5f, "reachable_viewpoints", moveit_interface->getPoseReferenceFrame());
        }

        // Execute preset joint configurations before planned viewpoints
        for (size_t pi = 0; !preset_configs.empty(); ++pi)
        {
            RCLCPP_INFO(node->get_logger(), "\n********** Preset Config %zu **********", pi);
            auto joint_config = preset_configs.front();
            preset_configs.pop_front();

            if (!moveit_interface->planToJointStateWithRetries(joint_config))
            {
                RCLCPP_WARN(node->get_logger(), "Failed to execute preset joint configuration %zu, skipping...", pi);
                continue;
            }

            waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

            if (config.enable_evaluation && octomap_interface->isSemanticTree())
            {
                EvaluationMetrics eval_metrics;
                eval_metrics.time = node->now().seconds() - initial_time;
                auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
                auto match_result = octomap_interface->matchClustersToGroundTruth(
                    latest_clusters, config.eval_threshold_radius, false);
                eval_metrics.class_metrics = octomap_interface->evaluateMatchResults(match_result, false);
                auto [occupied, free] = octomap_interface->getVoxelCounts();
                eval_metrics.occupied_voxels = occupied;
                eval_metrics.free_voxels = free;
                eval_metrics.bbox_coverage = octomap_interface->calculateCoverage();
                all_metrics.push_back(eval_metrics);
                RCLCPP_INFO(node->get_logger(), "\n=== Evaluation Results ===");
                RCLCPP_INFO(node->get_logger(), "Class ID | TP Clusters | FP Clusters | TP Points | FN Points");
                RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
                for (const auto &cm : eval_metrics.class_metrics)
                    RCLCPP_INFO(node->get_logger(), "  %6d | %11d | %12d | %9d | %9d",
                                cm.class_id, cm.tp_clusters, cm.fp_clusters, cm.tp_points, cm.fn_points);
                RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
                RCLCPP_INFO(node->get_logger(), "Class ID | Precision | Recall | F1 Score");
                RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
                for (const auto &cm : eval_metrics.class_metrics)
                    RCLCPP_INFO(node->get_logger(), "  %6d | %9.2f%% | %6.2f%% | %8.2f%%",
                                cm.class_id, cm.precision * 100.0, cm.recall * 100.0, cm.f1_score * 100.0);
                if (visualizer)
                    visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
                visualizer->logAllMetricsToCSV(all_metrics, config.metrics_data_dir);
            }

            if (visualizer)
                visualizer->clearAllMarkers();

            if (!rclcpp::ok())
            {
                RCLCPP_INFO(node->get_logger(), "\nNBV planning interrupted by shutdown signal, exiting...");
                return 1;
            }
        }

        // Main NBV Planning Loop
        for (size_t i = 0; i < reachable_viewpoints.size(); i++)
        {
            RCLCPP_INFO(node->get_logger(), "\n********** NBV Baseline Iteration %ld **********", i);

            auto plan = planPathsToViewpoint(reachable_viewpoints[i], moveit_interface, config, node->get_logger(), config.hint_joint_configs);
            if (!plan)
            {
                RCLCPP_WARN(node->get_logger(), "Failed to plan to the next viewpoint, skipping iteration...");
                continue;
            }

            if (!executeAndWaitForMotion(moveit_interface, node, *plan, node->get_logger()))
            {
                RCLCPP_ERROR(node->get_logger(), "Motion execution failed, ending NBV planning");
                break;
            }

            // Update the octomap after motion
            waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

            // Evaluate if enabled
            if (config.enable_evaluation && octomap_interface->isSemanticTree())
            {
                EvaluationMetrics eval_metrics;
                eval_metrics.time = node->now().seconds() - initial_time;

                auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
                auto match_result = octomap_interface->matchClustersToGroundTruth(
                    latest_clusters, config.eval_threshold_radius, false);

                eval_metrics.class_metrics = octomap_interface->evaluateMatchResults(match_result, false);

                auto [occupied, free] = octomap_interface->getVoxelCounts();
                eval_metrics.occupied_voxels = occupied;
                eval_metrics.free_voxels = free;

                eval_metrics.bbox_coverage = octomap_interface->calculateCoverage();
                all_metrics.push_back(eval_metrics);

                // Print results
                RCLCPP_INFO(node->get_logger(), "\n=== Evaluation Results ===");
                RCLCPP_INFO(node->get_logger(), "Class ID | TP Clusters | FP Clusters | TP Points | FN Points");
                RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
                for (const auto &cm : eval_metrics.class_metrics)
                    RCLCPP_INFO(node->get_logger(), "  %6d | %11d | %12d | %9d | %9d",
                                cm.class_id, cm.tp_clusters, cm.fp_clusters, cm.tp_points, cm.fn_points);
                RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
                RCLCPP_INFO(node->get_logger(), "Class ID | Precision | Recall | F1 Score");
                RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
                for (const auto &cm : eval_metrics.class_metrics)
                    RCLCPP_INFO(node->get_logger(), "  %6d | %9.2f%% | %6.2f%% | %8.2f%%",
                                cm.class_id, cm.precision * 100.0, cm.recall * 100.0, cm.f1_score * 100.0);

                if (visualizer)
                {
                    visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
                }
                // visualizer->plotAllMetrics(all_metrics, config.metrics_plots_dir);
                visualizer->logAllMetricsToCSV(all_metrics, config.metrics_data_dir);
            }

            // Clear visualization
            if (visualizer)
            {
                visualizer->clearAllMarkers();
            }

            if (!rclcpp::ok())
            {
                RCLCPP_INFO(node->get_logger(), "\nNBV planning interrupted by shutdown signal, exiting...");
                return 1;
            }
        }

        // Return to initial joint configuration
        // moveit_interface->clearPathConstraints();
        RCLCPP_INFO(node->get_logger(), "\nReturning to initial joint configuration...");
        if (!moveit_interface->planToJointStateWithRetries(config.init_joint_angles_rad))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Moved to initial joint configuration successfully");

        // Clear visualization at end of each run
        if (visualizer)
        {
            visualizer->clearAllMarkers();
        }

        // Reset constraints for next run (they’ll be set again at top of loop)
        moveit_interface->clearPathConstraints();
    }

    if (node->get_parameter("keep_alive").as_bool())
    {
        RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");
        while (rclcpp::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "moveit_interface_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    int result = run(node);

    rclcpp::shutdown();
    spinner.join();
    return result;
}