#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>
#include <queue>
#include <algorithm>
#include <optional>
#include <random>
#include <cmath>
#include <limits>
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

using namespace erwinia_os_nbv_planner;
using namespace erwinia_os_nbv_planner::conversions;

struct NBVPlannerConfig
{
    // Workspace Parameters
    std::string manipulator_group_name;
    bool learn_workspace;
    int num_samples;
    std::string workspace_file;

    // Octomap Parameters
    std::string octomap_topic;
    int min_unknown_neighbors;

    // Manipulation Parameters
    std::string planning_pipeline_id;
    std::string planner_id;
    double planning_time; // seconds
    int num_planning_attempts;
    double max_velocity_scaling_factor;
    double max_acceleration_scaling_factor;
    int num_ik_seeds;
    int plans_per_seed;
    double ik_timeout;
    int ik_attempts;

    // Camera Parameters
    std::string capture_type;
    std::string camera_optical_link;
    double camera_horizontal_fov_rad; // radians
    double camera_vertical_fov_rad;   // radians
    int camera_scaled_width;
    int camera_scaled_height;
    double camera_max_range;
    double ideal_camera_distance; // 0.3 m
    double ideal_distance_tolerance; // 0.1 m
    int num_camera_rays;

    // Evaluation Parameters
    bool enable_evaluation;
    double eval_threshold_radius;
    std::string gt_points_file;
    std::string metrics_plots_dir;
    std::string metrics_data_dir;

    // General Parameters
    std::vector<double> init_joint_angles_rad;
    std::vector<std::vector<double>> hint_joint_configs; // optional IK seed hints (rad)
    std::string map_frame;

    // NBV Planning Parameters
    int max_iterations;
    double min_information_gain;
    double alpha_cost_weight;
    double beta_semantic_weight;
    double conf_thresh;
    double semantic_confidence_boost;
    double semantic_mismatch_penalty;
    // Voxels with confidence above this are considered fully certain and are
    // excluded from getUncertainVoxels (replaces conf_thresh + boost in that call)
    double max_semantic_certainty;

    // Viewpoint Parameters
    double plane_half_extent;
    double plane_spatial_resolution;
    double cap_max_theta_rad;
    double cap_min_theta_rad;
    int num_viewpoints_per_frontier;
    double z_bias_sigma;             // M_PI / 3.0
    double viewpoint_overlap_ratio;

    // Debug Parameters
    bool visualize;
    std::string visualization_topic;
};

struct TriggerClients
{
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_video;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_video;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr send_trigger;
};

// ============================================================================
// Helper Functions
// ============================================================================
void waitForSimClock(const std::shared_ptr<rclcpp::Node> &node)
{
    RCLCPP_INFO(node->get_logger(), "Waiting for /clock...");
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, [](const rosgraph_msgs::msg::Clock::SharedPtr) {});

    rclcpp::Rate rate(10);
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        if (node->now().seconds() > 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Clock synchronized at %.2f seconds",
                        node->now().seconds());
            break;
        }
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::seconds(10))
        {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting for /clock. Proceeding anyway...");
            break;
        }
        rate.sleep();
    }
}

NBVPlannerConfig loadConfiguration(const std::shared_ptr<rclcpp::Node> &node)
{
    NBVPlannerConfig config;

    // Workspace Parameters
    config.manipulator_group_name = node->get_parameter("manipulator_group_name").as_string();
    config.learn_workspace = node->get_parameter("learn_workspace").as_bool();
    config.num_samples = node->get_parameter("num_samples").as_int();
    config.workspace_file = node->get_parameter("manipulation_workspace_file").as_string();

    // Octomap Parameters
    config.octomap_topic = node->get_parameter("octomap_topic").as_string();
    config.min_unknown_neighbors = node->get_parameter("min_unknown_neighbors").as_int();

    // Manipulation Parameters
    config.planning_pipeline_id = node->get_parameter("planning_pipeline_id").as_string();
    config.planner_id = node->get_parameter("planner_id").as_string();
    config.planning_time = node->get_parameter("planning_time").as_double();
    config.num_planning_attempts = node->get_parameter("num_planning_attempts").as_int();
    config.max_velocity_scaling_factor = node->get_parameter("max_velocity_scaling_factor").as_double();
    config.max_acceleration_scaling_factor = node->get_parameter("max_acceleration_scaling_factor").as_double();
    config.num_ik_seeds = node->get_parameter("num_ik_seeds").as_int();
    config.plans_per_seed = node->get_parameter("plans_per_seed").as_int();
    config.ik_timeout = node->get_parameter("ik_timeout").as_double();
    config.ik_attempts = node->get_parameter("ik_attempts").as_int();

    // Camera Parameters
    config.capture_type = node->get_parameter("capture_type").as_string();
    config.camera_optical_link = node->get_parameter("camera_optical_link").as_string();
    config.camera_horizontal_fov_rad = geometry_utils::deg2Rad(node->get_parameter("camera_horizontal_fov_deg").as_double());
    config.camera_vertical_fov_rad = geometry_utils::deg2Rad(node->get_parameter("camera_vertical_fov_deg").as_double());
    config.camera_scaled_width = node->get_parameter("camera_scaled_width").as_int();
    config.camera_scaled_height = node->get_parameter("camera_scaled_height").as_int();
    config.camera_max_range = node->get_parameter("camera_max_range").as_double();
    config.ideal_camera_distance = node->get_parameter("ideal_camera_distance").as_double();
    config.ideal_distance_tolerance = node->get_parameter("ideal_distance_tolerance").as_double();
    config.num_camera_rays = node->get_parameter("num_camera_rays").as_int();

    // Evaluation Parameters
    config.enable_evaluation = node->get_parameter("enable_evaluation").as_bool();
    config.eval_threshold_radius = node->get_parameter("eval_threshold_radius").as_double();
    config.gt_points_file = node->get_parameter("gt_points_file").as_string();
    config.metrics_plots_dir = node->get_parameter("metrics_plots_dir").as_string();
    config.metrics_data_dir = node->get_parameter("metrics_data_dir").as_string();

    // General Parameters
    config.init_joint_angles_rad = geometry_utils::deg2Rad(node->get_parameter("init_joint_angles_deg").as_double_array());

    // Optional flat list of hint joint configs (groups of N joints, in degrees)
    if (node->has_parameter("hint_joint_angles_deg"))
    {
        auto flat = node->get_parameter("hint_joint_angles_deg").as_double_array();
        const size_t n = config.init_joint_angles_rad.size();
        if (n > 0 && !flat.empty() && flat.size() % n == 0)
        {
            for (size_t i = 0; i < flat.size(); i += n)
            {
                std::vector<double> hint_deg(flat.begin() + i, flat.begin() + i + n);
                config.hint_joint_configs.push_back(geometry_utils::deg2Rad(hint_deg));
            }
        }
        else if (!flat.empty())
        {
            RCLCPP_WARN(node->get_logger(),
                "hint_joint_angles_deg size %zu is not a multiple of num_joints %zu, ignoring hints.",
                flat.size(), n);
        }
    }

    config.map_frame = node->get_parameter("map_frame").as_string();

    // NBV Planning Parameters
    config.max_iterations = node->get_parameter("max_iterations").as_int();
    config.min_information_gain = node->get_parameter("min_information_gain").as_double();
    config.alpha_cost_weight = node->get_parameter("alpha_cost_weight").as_double();
    config.beta_semantic_weight = node->get_parameter("beta_semantic_weight").as_double();
    config.conf_thresh = node->get_parameter("conf_thresh").as_double();
    config.semantic_confidence_boost = node->get_parameter("semantic_confidence_boost").as_double();
    config.semantic_mismatch_penalty = node->get_parameter("semantic_mismatch_penalty").as_double();
    config.max_semantic_certainty = node->get_parameter("max_semantic_certainty").as_double();

    // Viewpoint Parameters
    config.plane_half_extent = node->get_parameter("plane_half_extent").as_double();
    config.plane_spatial_resolution = node->get_parameter("plane_spatial_resolution").as_double();
    config.cap_max_theta_rad = geometry_utils::deg2Rad(node->get_parameter("cap_max_theta_deg").as_double());
    config.cap_min_theta_rad = geometry_utils::deg2Rad(node->get_parameter("cap_min_theta_deg").as_double());
    config.num_viewpoints_per_frontier = node->get_parameter("num_viewpoints_per_frontier").as_int();
    config.z_bias_sigma = node->get_parameter("z_bias_sigma").as_double();
    config.viewpoint_overlap_ratio = node->get_parameter("viewpoint_overlap_ratio").as_double();

    // Debug Parameters
    config.visualize = node->get_parameter("visualize").as_bool();
    config.visualization_topic = node->get_parameter("visualization_topic").as_string();

    return config;
}

void printConfiguration(const NBVPlannerConfig &config, const rclcpp::Logger &logger)
{
    RCLCPP_INFO(logger, "\n=== NBV Baseline Configuration ===");

    // Workspace Parameters
    RCLCPP_INFO(logger, "--- Workspace ---");
    RCLCPP_INFO(logger, "  Manipulator group: %s", config.manipulator_group_name.c_str());
    RCLCPP_INFO(logger, "  Learn workspace: %s", config.learn_workspace ? "true" : "false");
    RCLCPP_INFO(logger, "  Workspace samples: %d", config.num_samples);
    RCLCPP_INFO(logger, "  Workspace file: %s", config.workspace_file.c_str());

    // Octomap Parameters
    RCLCPP_INFO(logger, "--- Octomap ---");
    RCLCPP_INFO(logger, "  Octomap topic: %s", config.octomap_topic.c_str());
    RCLCPP_INFO(logger, "  Min unknown neighbors: %d", config.min_unknown_neighbors);

    // Manipulation Parameters
    RCLCPP_INFO(logger, "--- Manipulation ---");
    RCLCPP_INFO(logger, "  Planning pipeline: %s", config.planning_pipeline_id.c_str());
    RCLCPP_INFO(logger, "  Planner ID: %s", config.planner_id.c_str());
    RCLCPP_INFO(logger, "  Planning time: %.2f s", config.planning_time);
    RCLCPP_INFO(logger, "  Planning attempts: %d", config.num_planning_attempts);
    RCLCPP_INFO(logger, "  Max velocity scale: %.2f", config.max_velocity_scaling_factor);
    RCLCPP_INFO(logger, "  Max accel scale: %.2f", config.max_acceleration_scaling_factor);
    RCLCPP_INFO(logger, "  IK seeds: %d", config.num_ik_seeds);
    RCLCPP_INFO(logger, "  Plans per seed: %d", config.plans_per_seed);
    RCLCPP_INFO(logger, "  IK timeout: %.2f s", config.ik_timeout);
    RCLCPP_INFO(logger, "  IK attempts: %d", config.ik_attempts);

    // Camera Parameters
    RCLCPP_INFO(logger, "--- Camera ---");
    RCLCPP_INFO(logger, "  Capture type: %s", config.capture_type.c_str());
    RCLCPP_INFO(logger, "  Optical link: %s", config.camera_optical_link.c_str());
    RCLCPP_INFO(logger, "  H-FOV: %.1f rad (%.1f deg)", config.camera_horizontal_fov_rad, geometry_utils::rad2Deg(config.camera_horizontal_fov_rad));
    RCLCPP_INFO(logger, "  V-FOV: %.1f rad (%.1f deg)", config.camera_vertical_fov_rad, geometry_utils::rad2Deg(config.camera_vertical_fov_rad));
    RCLCPP_INFO(logger, "  Scaled Resolution: %dx%d", config.camera_scaled_width, config.camera_scaled_height);
    RCLCPP_INFO(logger, "  Max range: %.2f m", config.camera_max_range);
    RCLCPP_INFO(logger, "  Ideal distance: %.2f m", config.ideal_camera_distance);
    RCLCPP_INFO(logger, "  Ideal distance tolerance: %.4f", config.ideal_distance_tolerance);
    RCLCPP_INFO(logger, "  Number of IG rays: %d", config.num_camera_rays);

    // Evaluation Parameters
    RCLCPP_INFO(logger, "--- Evaluation ---");
    RCLCPP_INFO(logger, "  Enable evaluation: %s", config.enable_evaluation ? "true" : "false");
    RCLCPP_INFO(logger, "  Threshold radius: %.2f m", config.eval_threshold_radius);
    RCLCPP_INFO(logger, "  GT points file: %s", config.gt_points_file.c_str());
    RCLCPP_INFO(logger, "  Metrics plots dir: %s", config.metrics_plots_dir.c_str());
    RCLCPP_INFO(logger, "  Metrics data dir: %s", config.metrics_data_dir.c_str());

    // General Parameters
    RCLCPP_INFO(logger, "--- General ---");
    RCLCPP_INFO(logger, "  Initial joint angles (rad):");
    for (size_t i = 0; i < config.init_joint_angles_rad.size(); ++i)
    {
        RCLCPP_INFO(logger, "    Joint %zu: %.4f", i, config.init_joint_angles_rad[i]);
    }
    RCLCPP_INFO(logger, "  Map frame: %s", config.map_frame.c_str());

    // NBV Planning Parameters
    RCLCPP_INFO(logger, "--- NBV Planning ---");
    RCLCPP_INFO(logger, "  Max iterations: %d", config.max_iterations);
    RCLCPP_INFO(logger, "  Min information gain: %.4f", config.min_information_gain);
    RCLCPP_INFO(logger, "  Alpha cost weight: %.4f", config.alpha_cost_weight);
    RCLCPP_INFO(logger, "  Beta semantic weight: %.4f", config.beta_semantic_weight);
    RCLCPP_INFO(logger, "  Confidence threshold: %.4f", config.conf_thresh);
    RCLCPP_INFO(logger, "  Semantic confidence boost: %.4f", config.semantic_confidence_boost);
    RCLCPP_INFO(logger, "  Semantic mismatch penalty: %.4f", config.semantic_mismatch_penalty);
    RCLCPP_INFO(logger, "  Max semantic certainty: %.4f", config.max_semantic_certainty);

    // Viewpoint Parameters
    RCLCPP_INFO(logger, "--- Viewpoints ---");
    RCLCPP_INFO(logger, "  Plane half extent: %.2f m", config.plane_half_extent);
    RCLCPP_INFO(logger, "  Plane spatial resolution: %.2f m", config.plane_spatial_resolution);
    RCLCPP_INFO(logger, "  Cap max theta: %.1f rad (%.1f deg)", config.cap_max_theta_rad, geometry_utils::rad2Deg(config.cap_max_theta_rad));
    RCLCPP_INFO(logger, "  Cap min theta: %.1f rad (%.1f deg)", config.cap_min_theta_rad, geometry_utils::rad2Deg(config.cap_min_theta_rad));
    RCLCPP_INFO(logger, "  Viewpoints per frontier: %d", config.num_viewpoints_per_frontier);
    RCLCPP_INFO(logger, "  Z-bias sigma: %.4f", config.z_bias_sigma);
    RCLCPP_INFO(logger, "  Viewpoint overlap ratio: %.4f", config.viewpoint_overlap_ratio);

    // Debug Parameters
    RCLCPP_INFO(logger, "--- Debug ---");
    RCLCPP_INFO(logger, "  Visualization: %s", config.visualize ? "enabled" : "disabled");
    RCLCPP_INFO(logger, "  Visualization topic: %s", config.visualization_topic.c_str());

    RCLCPP_INFO(logger, "====================================\n");
}

std::shared_ptr<MoveItInterface> setupMoveItInterface(
    const std::shared_ptr<rclcpp::Node> &node,
    const NBVPlannerConfig &config)
{
    auto interface = std::make_shared<MoveItInterface>(node, config.manipulator_group_name);
    interface->setPlanningPipelineId(config.planning_pipeline_id);
    interface->setPlannerId(config.planner_id);
    interface->setPlanningTime(config.planning_time);
    interface->setNumPlanningAttempts(config.num_planning_attempts);
    interface->setMaxVelocityScalingFactor(config.max_velocity_scaling_factor);
    interface->setMaxAccelerationScalingFactor(config.max_acceleration_scaling_factor);
    return interface;
}

std::shared_ptr<ManipulationWorkspace> setupWorkspace(
    const std::shared_ptr<MoveItInterface> &interface,
    const std::shared_ptr<NBVVisualizer> &visualizer,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger)
{
    auto workspace = std::make_shared<ManipulationWorkspace>(interface, visualizer);

    if (config.learn_workspace)
    {
        RCLCPP_INFO(logger, "Learning workspace with %d samples...", config.num_samples);
        if (!workspace->learnWorkspace(config.num_samples, config.visualize))
        {
            RCLCPP_ERROR(logger, "Failed to learn workspace");
            return nullptr;
        }
        RCLCPP_INFO(logger, "Learned %zu voxels, saving to: %s",
                    workspace->getNumReachableVoxels(), config.workspace_file.c_str());
        workspace->saveWorkspaceToFile(config.workspace_file);
    }
    else
    {
        RCLCPP_DEBUG(logger, "Loading workspace from: %s", config.workspace_file.c_str());
        if (!workspace->loadWorkspaceFromFile(config.workspace_file))
        {
            RCLCPP_ERROR(logger, "Failed to load. Set learn_workspace:=true");
            return nullptr;
        }
        RCLCPP_DEBUG(logger, "Loaded %zu voxels", workspace->getNumReachableVoxels());
    }

    return workspace;
}

TriggerClients createTriggerClients(const std::shared_ptr<rclcpp::Node> &node)
{
    return {
        node->create_client<std_srvs::srv::Trigger>("/trigger/start_video"),
        node->create_client<std_srvs::srv::Trigger>("/trigger/stop_video"),
        node->create_client<std_srvs::srv::Trigger>("/trigger/send_trigger")};
}

bool callClearMap(
    const std::shared_ptr<rclcpp::Node> & /*node*/,
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &clear_client,
    const rclcpp::Logger &logger)
{
    if (!clear_client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger, "Clear service /occupancy_map/clear not available");
        return false;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clear_client->async_send_request(req);

    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger, "Failed to call /occupancy_map/clear (timeout)");
        return false;
    }

    auto resp = future.get();
    if (!resp->success)
    {
        RCLCPP_ERROR(logger, "Clear service returned failure: %s", resp->message.c_str());
        return false;
    }

    RCLCPP_INFO(logger, "Occupancy map cleared: %s", resp->message.c_str());
    return true;
}

bool startContinuousCapture(
    const std::shared_ptr<rclcpp::Node> & /*node*/,
    const TriggerClients &clients,
    const rclcpp::Logger &logger)
{
    if (!clients.start_video->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(logger, "start_video service not available, continuing anyway...");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clients.start_video->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_DEBUG(logger, "Continuous video capture started: %s", response->message.c_str());
            return true;
        }
        else
        {
            RCLCPP_WARN(logger, "Failed to start video: %s", response->message.c_str());
        }
    }
    return false;
}

bool stopVideoCapture(
    const std::shared_ptr<rclcpp::Node> & /*node*/,
    const TriggerClients &clients,
    const rclcpp::Logger &logger)
{
    if (!clients.stop_video->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(logger, "stop_video service not available, continuing anyway...");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clients.stop_video->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto response = future.get();
        RCLCPP_DEBUG(logger, "Stop video service called: %s", response->message.c_str());
        return true;
    }
    return false;
}

bool waitForOctomapWithTriggers(
    const std::shared_ptr<rclcpp::Node> &node,
    const std::shared_ptr<OctoMapInterface> &octomap_interface,
    const TriggerClients &clients,
    const NBVPlannerConfig &config,
    double timeout_seconds,
    const rclcpp::Logger &logger)
{
    rclcpp::Time start_time = node->now();
    rclcpp::Rate spin_rate(10);
    int trigger_count = 0;
    auto last_trigger_time = node->now();

    RCLCPP_DEBUG(logger, "Waiting for octomap (timeout: %.0f seconds)", timeout_seconds);

    while (rclcpp::ok() && !octomap_interface->isTreeAvailable())
    {
        // Check timeout
        if ((node->now() - start_time).seconds() >= timeout_seconds)
        {
            RCLCPP_ERROR(logger, "Octomap not available after %.0f seconds. Exiting.", timeout_seconds);
            return false;
        }

        // Send trigger every 1 second in triggered mode
        if (config.capture_type == "triggered" && (node->now() - last_trigger_time).seconds() >= 1.0)
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = clients.send_trigger->async_send_request(request);
            trigger_count++;
            last_trigger_time = node->now();
            RCLCPP_DEBUG(logger, "Sending camera trigger #%d...", trigger_count);
        }

        spin_rate.sleep();
    }

    RCLCPP_DEBUG(logger, "Octomap received successfully");
    return true;
}

void waitForOctomap(
    const std::shared_ptr<rclcpp::Node> &node,
    const std::shared_ptr<OctoMapInterface> &octomap_interface,
    const TriggerClients &clients,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger)
{
    rclcpp::Rate spin_rate(10);
    int trigger_count = 0;
    auto last_trigger_time = node->now();

    // Capture initial timestamp (will be 0.0 if no octomap exists yet)
    rclcpp::Time initial_time = octomap_interface->getLastUpdateTime();
    double initial_time_seconds = initial_time.seconds();

    RCLCPP_DEBUG(logger, "Waiting for octomap (initial timestamp: %.3f seconds)", initial_time_seconds);

    while (rclcpp::ok())
    {
        // Send trigger every 1 second in triggered mode
        if (config.capture_type == "triggered" && (node->now() - last_trigger_time).seconds() >= 1.0)
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = clients.send_trigger->async_send_request(request);
            trigger_count++;
            last_trigger_time = node->now();
            RCLCPP_DEBUG(logger, "Sending camera trigger #%d...", trigger_count);
        }

        // Check if octomap is available AND has been updated
        if (octomap_interface->isTreeAvailable())
        {
            rclcpp::Time current_time = octomap_interface->getLastUpdateTime();
            double current_time_seconds = current_time.seconds();

            if (current_time_seconds != initial_time_seconds)
            {
                RCLCPP_DEBUG(logger, "Octomap available and updated (timestamp: %.3f seconds)", current_time_seconds);
                break;
            }
        }

        spin_rate.sleep();
    }
}

/**
 * @brief Get transform from MoveIt planning frame to OctoMap frame.
 * Returns identity transform if frames are the same.
 */
geometry_msgs::msg::TransformStamped getMoveitToOctomapTransform(
    const std::shared_ptr<MoveItInterface> &moveit_interface,
    const std::shared_ptr<OctoMapInterface> &octomap_interface,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const rclcpp::Logger &logger)
{
    std::string moveit_frame = moveit_interface->getPoseReferenceFrame();
    std::string octomap_frame = octomap_interface->getOctomapFrameId();

    geometry_msgs::msg::TransformStamped transform;
    if (moveit_frame != octomap_frame)
    {
        try
        {
            transform = tf_buffer->lookupTransform(
                octomap_frame,
                moveit_frame,
                tf2::TimePointZero);
            RCLCPP_DEBUG(logger, "Transform from %s to %s found",
                         moveit_frame.c_str(), octomap_frame.c_str());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(logger, "Failed to get transform from %s to %s: %s",
                         moveit_frame.c_str(), octomap_frame.c_str(), ex.what());
            throw;
        }
    }
    else
    {
        RCLCPP_DEBUG(logger, "MoveIt frame and OctoMap frame are the same: %s", moveit_frame.c_str());
        // Create identity transform
        transform.header.frame_id = moveit_frame;
        transform.child_frame_id = octomap_frame;
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
    }
    return transform;
}

// ============================================================================
// NBV Planner Functions
// ============================================================================

/**
 * @brief Filter viewpoints to retain only those that are reachable by the manipulator.
 * @viewpoints: Candidate eef poses in moveit frame
 */
std::vector<Viewpoint> filterReachableViewpoints(
    const std::vector<Viewpoint> &viewpoints,
    const std::shared_ptr<ManipulationWorkspace> &manip_workspace,
    const std::shared_ptr<MoveItInterface> &moveit_interface,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger)
{
    // Filter by reachability
    std::vector<Viewpoint> reachable_viewpoints;
    for (const auto &vp : viewpoints)
    {
        // Convert to end-effector pose
        geometry_msgs::msg::Pose cam_pose = eigenToPose(vp.position, vp.orientation);
        geometry_msgs::msg::Pose ee_pose;
        if (!moveit_interface->cameraPoseToEEPose(cam_pose, config.camera_optical_link, ee_pose))
        {
            RCLCPP_INFO(logger, "Failed to convert camera pose to EE pose for reachability check...");
            continue;
        }
        if (manip_workspace->isPoseReachable(ee_pose))
        {
            reachable_viewpoints.push_back(vp);
        }
    }
    RCLCPP_DEBUG(logger, "%zu out of %zu viewpoints are reachable",
                 reachable_viewpoints.size(), viewpoints.size());

    return reachable_viewpoints;
}

/**
 * @brief Sort the viewpoints based on coordinate values
 * @viewpoints: Candidate viewpoints
 * @axis_priority: Vector indicating axis priority for sorting (e.g., {"x","y","z"} or {"-y","x"} for descending priority on y)
 */
void sortViewpointsByAxisPriority(
    std::vector<Viewpoint> &viewpoints,
    const std::vector<std::string> &axis_priority)
{
    std::sort(viewpoints.begin(), viewpoints.end(),
              [&axis_priority](const Viewpoint &a, const Viewpoint &b)
              {
                  for (const auto &axis_str : axis_priority)
                  {
                      // Skip invalid axis (empty string)
                      if (axis_str.empty())
                      {
                          continue;
                      }

                      // Determine axis and sort order
                      double val_a, val_b;
                      if (axis_str == "-x")
                      {
                          val_a = -a.position.x();
                          val_b = -b.position.x();
                      }
                      else if (axis_str == "x")
                      {
                          val_a = a.position.x();
                          val_b = b.position.x();
                      }
                      else if (axis_str == "-y")
                      {
                          val_a = -a.position.y();
                          val_b = -b.position.y();
                      }
                      else if (axis_str == "y")
                      {
                          val_a = a.position.y();
                          val_b = b.position.y();
                      }
                      else if (axis_str == "-z")
                      {
                          val_a = -a.position.z();
                          val_b = -b.position.z();
                      }
                      else if (axis_str == "z")
                      {
                          val_a = a.position.z();
                          val_b = b.position.z();
                      }
                      else
                      {
                          continue; // Invalid axis, skip
                      }

                      // Check if values are different (with small epsilon for floating point)
                      if (std::abs(val_a - val_b) > 1e-6)
                          return val_a < val_b;
                  }
                  return false; // All axes equal
              });
}

void lawnmowerSortViewpoints(
    std::vector<Viewpoint> &viewpoints)
{
    // Determine if viewpoints are on YZ or XZ plane by checking x coordinates
    bool is_xz_plane = false;
    for (size_t i = 1; i < viewpoints.size(); ++i)
    {
        is_xz_plane = (std::abs(viewpoints[0].position.x() - viewpoints[i].position.x()) > 1e-6);
        if (is_xz_plane) break;
    }
    // Sort by primary axis z (descending), then secondary axis y or x (descending)
    if (is_xz_plane)
        sortViewpointsByAxisPriority(viewpoints, {"-z", "-x"});
    else
        sortViewpointsByAxisPriority(viewpoints, {"-z", "-y"});
    // Now reverse every other row to create lawnmower pattern
    size_t row_start = 0;
    size_t row_end = 1;
    size_t row_count = 1;
    std::vector<Viewpoint> sorted_viewpoints;
    while (row_start < viewpoints.size())
    {
        std::vector<Viewpoint> row;
        while (row_end < viewpoints.size())
        {
            bool same_row = (std::abs(viewpoints[row_end].position.z() - viewpoints[row_start].position.z()) < 1e-6);
            if (same_row)
                row_end++;
            else
                break;
        }
        // Reverse every other row
        if (row_count % 2 == 0)
            std::reverse(viewpoints.begin() + row_start, viewpoints.begin() + row_end);
        row_start = row_end;
        row_count++;
    }
}

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planPathToViewpoint(
    const Viewpoint &viewpoint,
    const std::shared_ptr<MoveItInterface> &moveit_interface,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger)
{
    std::vector<double> current_joint_state;
    if (!moveit_interface->getCurrentJointAngles(current_joint_state))
    {
        RCLCPP_ERROR(logger, "Failed to get current joint state for IK seed");
        return std::nullopt;
    }

    // Convert camera pose to end-effector pose
    geometry_msgs::msg::Pose cam_pose = eigenToPose(viewpoint.position, viewpoint.orientation);
    geometry_msgs::msg::Pose target_ee_pose;
    if (!moveit_interface->cameraPoseToEEPose(cam_pose, config.camera_optical_link, target_ee_pose))
    {
        RCLCPP_INFO(logger, "Failed to convert camera pose to EE pose");
        return std::nullopt;
    }

    // Compute IK
    auto next_joint_angles = moveit_interface->computeIK(current_joint_state, target_ee_pose, 0.5, 10);
    if (next_joint_angles.empty())
    {
        RCLCPP_INFO(logger, "No IK solution found for this viewpoint");
        return std::nullopt;
    }

    // Check collision-free
    if (!moveit_interface->isStateValid(next_joint_angles))
    {
        RCLCPP_INFO(logger, "IK solution is in collision");
        return std::nullopt;
    }

    // Plan path
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_interface->planToJointGoal(next_joint_angles, plan);
    if (plan.trajectory_.joint_trajectory.points.empty())
    {
        RCLCPP_INFO(logger, "No reachable path to IK solution");
        return std::nullopt;
    }

    return plan;
}

/**
 * @brief Plan path to viewpoint using multiple IK seeds and retry logic.
 * This is a simplified wrapper around MoveItInterface::planToTargetPoseWithRetries.
 */
std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planPathsToViewpoint(
    const Viewpoint &viewpoint,
    const std::shared_ptr<MoveItInterface> &moveit_interface,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger,
    const std::vector<std::vector<double>> &hint_seeds = {})
{
    // Convert camera pose to end-effector pose
    geometry_msgs::msg::Pose cam_pose = eigenToPose(viewpoint.position, viewpoint.orientation);
    geometry_msgs::msg::Pose target_ee_pose;
    if (!moveit_interface->cameraPoseToEEPose(cam_pose, config.camera_optical_link, target_ee_pose))
    {
        RCLCPP_INFO(logger, "Failed to convert camera pose to EE pose");
        return std::nullopt;
    }

    // Use MoveItInterface's advanced planning with retries
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = moveit_interface->planToTargetPoseWithRetries(
        target_ee_pose,
        plan,
        config.num_ik_seeds,
        config.plans_per_seed,
        config.ik_timeout,
        config.ik_attempts,
        hint_seeds);

    if (!success)
    {
        RCLCPP_INFO(logger, "No valid plan found for viewpoint after retries");
        return std::nullopt;
    }

    return plan;
}

bool executeAndWaitForMotion(
    const std::shared_ptr<MoveItInterface> &moveit_interface,
    const std::shared_ptr<rclcpp::Node> & /*node*/,
    const moveit::planning_interface::MoveGroupInterface::Plan &plan,
    const rclcpp::Logger &logger)
{
    if (!moveit_interface->execute(plan))
    {
        RCLCPP_ERROR(logger, "Failed to execute motion to best viewpoint!");
        return false;
    }

    // Extract target joint angles from plan
    std::vector<double> target_joint_angles = plan.trajectory_.joint_trajectory.points.back().positions;

    // Wait for motion to complete
    rclcpp::Rate spin_rate(10);
    RCLCPP_DEBUG(logger, "Waiting for manipulator to reach goal");
    while (rclcpp::ok())
    {
        std::vector<double> current_joints;
        if (moveit_interface->getCurrentJointAngles(current_joints))
        {
            double joint_error = 0.0;
            for (size_t i = 0; i < current_joints.size() && i < target_joint_angles.size(); ++i)
            {
                joint_error += std::abs(current_joints[i] - target_joint_angles[i]);
            }
            if (joint_error < 0.01)
            {
                RCLCPP_DEBUG(logger, "Manipulator reached goal position");
                return true;
            }
        }
        spin_rate.sleep();
    }
    return false;
}
