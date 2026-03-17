/**
 * @file moveit_octomap_server.cpp
 * @brief MoveIt-integrated occupancy map server with octomap_msgs publishing
 */

#include "erwinia_os_occupancy_map/occupancy_map_monitor.hpp"
#include "erwinia_os_occupancy_map/pointcloud_updater.hpp"
#include "erwinia_os_occupancy_map/semantic_pointcloud_updater.hpp"
#include "erwinia_os_occupancy_map/occupancy_map_visualizer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <octomap_msgs/msg/octomap_with_pose.hpp>
#include "erwinia_os_occupancy_map/msg/custom_octomap.hpp"

using namespace erwinia_os_occupancy_map;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("octomap_server");

    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "Starting octomap server...");

    // Create TF buffer and listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Declare all parameters
    node->declare_parameter("resolution", 0.1);
    node->declare_parameter("map_frame", "map");
    node->declare_parameter("max_range", 3.0);
    node->declare_parameter("min_range", 0.0);
    node->declare_parameter("prob_hit", 0.7);
    node->declare_parameter("prob_miss", 0.4);
    node->declare_parameter("clamp_min", 0.1192);
    node->declare_parameter("clamp_max", 0.971);
    node->declare_parameter("occupancy_threshold", 0.5);
    // node->declare_parameter("filter_ground_plane", true);
    // node->declare_parameter("ground_distance_threshold", 0.04);

    node->declare_parameter("pointcloud_topic", "/camera/depth/points");
    node->declare_parameter("use_moveit", false);
    node->declare_parameter("planning_scene_world_topic", "/planning_scene_world");

    node->declare_parameter("publish_free_voxels", true);
    node->declare_parameter("enable_visualization", true);
    node->declare_parameter("visualization_topic", "occupancy_map_markers");
    node->declare_parameter("visualization_rate", 1.0);
    node->declare_parameter("octomap_publish_rate", 1.0);
    
    // Bounding box parameters
    node->declare_parameter("use_bounding_box", false);
    node->declare_parameter("bbx_min_x", -1.0);
    node->declare_parameter("bbx_min_y", -1.0);
    node->declare_parameter("bbx_min_z", -1.0);
    node->declare_parameter("bbx_max_x", 1.0);
    node->declare_parameter("bbx_max_y", 1.0);
    node->declare_parameter("bbx_max_z", 1.0);

    node->declare_parameter("use_semantics", false);
    node->declare_parameter("nbv_octomap_topic", "/octomap_binary");
    node->declare_parameter("nbv_octomap_qos_transient_local", true);

    // Semantic fusion parameters
    node->declare_parameter("semantic_confidence_boost", 0.05);
    node->declare_parameter("semantic_mismatch_penalty", 0.1);

    // Get parameters (already declared by automatically_declare_parameters_from_overrides)
    OccupancyMapParameters params;
    params.resolution = node->get_parameter("resolution").as_double();
    params.max_range = node->get_parameter("max_range").as_double();
    params.min_range = node->get_parameter("min_range").as_double();
    params.prob_hit = node->get_parameter("prob_hit").as_double();
    params.prob_miss = node->get_parameter("prob_miss").as_double();
    params.clamp_min = node->get_parameter("clamp_min").as_double();
    params.clamp_max = node->get_parameter("clamp_max").as_double();
    params.occupancy_threshold = node->get_parameter("occupancy_threshold").as_double();
    params.map_frame = node->get_parameter("map_frame").as_string();
    // params.filter_ground_plane = node->get_parameter("filter_ground_plane").as_bool();
    // params.ground_distance_threshold = node->get_parameter("ground_distance_threshold").as_double();

    std::string pointcloud_topic = node->get_parameter("pointcloud_topic").as_string();
    bool use_moveit = node->get_parameter("use_moveit").as_bool();
    std::string planning_scene_world_topic =
        node->get_parameter("planning_scene_world_topic").as_string();

    bool publish_free = node->get_parameter("publish_free_voxels").as_bool();
    bool enable_viz = node->get_parameter("enable_visualization").as_bool();
    std::string viz_topic = node->get_parameter("visualization_topic").as_string();
    double viz_rate = node->get_parameter("visualization_rate").as_double();

    params.use_bounding_box = node->get_parameter("use_bounding_box").as_bool();
    if (params.use_bounding_box)
    {
        params.bbx_min = octomap::point3d(
            node->get_parameter("bbx_min_x").as_double(),
            node->get_parameter("bbx_min_y").as_double(),
            node->get_parameter("bbx_min_z").as_double());
        params.bbx_max = octomap::point3d(
            node->get_parameter("bbx_max_x").as_double(),
            node->get_parameter("bbx_max_y").as_double(),
            node->get_parameter("bbx_max_z").as_double());
    }

    double octomap_publish_rate = node->get_parameter("octomap_publish_rate").as_double();
    std::string nbv_octomap_topic = node->get_parameter("nbv_octomap_topic").as_string();
    bool nbv_octomap_qos_transient_local = node->get_parameter("nbv_octomap_qos_transient_local").as_bool();

    bool use_semantics = node->get_parameter("use_semantics").as_bool();
    params.semantic_confidence_boost = node->get_parameter("semantic_confidence_boost").as_double();
    params.semantic_mismatch_penalty = node->get_parameter("semantic_mismatch_penalty").as_double();

    RCLCPP_INFO(logger, "Octomap mode: %s", use_semantics ? "SEMANTIC" : "STANDARD");
    // Create monitor (creates appropriate tree type internally)
    auto monitor = std::make_shared<OccupancyMapMonitor>(node, tf_buffer, params, use_semantics);

    if (use_semantics)
    {
        // ==================== SEMANTIC MODE ====================
        // Create semantic pointcloud updater and add to monitor
        auto semantic_updater = std::make_shared<SemanticPointCloudUpdater>(pointcloud_topic, params);
        semantic_updater->initialize(node, tf_buffer);
        semantic_updater->setSemanticTree(monitor->getSemanticMapTree());
        monitor->addUpdater(semantic_updater);

        RCLCPP_INFO(logger, "Using SemanticPointCloudUpdater on: %s", pointcloud_topic.c_str());

        // Create visualizer (supports both tree types via overloaded constructor)
        std::shared_ptr<OccupancyMapVisualizer> visualizer;
        if (enable_viz)
        {
            visualizer = std::make_shared<OccupancyMapVisualizer>(
                node, monitor->getSemanticMapTree(), params.map_frame, viz_topic);
            visualizer->setUpdateRate(viz_rate);
        }

        // Octomap publisher for planning scene world updates
        rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr psw_pub;
        if (use_moveit)
        {
            psw_pub = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
                planning_scene_world_topic, rclcpp::QoS(1).transient_local());
            RCLCPP_INFO(logger, "Planning scene world publisher created on topic: %s", planning_scene_world_topic.c_str());
        }

        // Create NBV octomap publisher
        rclcpp::QoS nbv_qos = rclcpp::QoS(1);
        if (nbv_octomap_qos_transient_local)
        {
            nbv_qos = rclcpp::QoS(1).transient_local();
        }
        auto nbv_pub = node->create_publisher<erwinia_os_occupancy_map::msg::CustomOctomap>(
            nbv_octomap_topic, nbv_qos);
        RCLCPP_INFO(logger, "NBV octomap publisher created on topic: %s", nbv_octomap_topic.c_str());

        auto last_octomap_publish = std::make_shared<rclcpp::Time>(node->now());

        // Set update callback on monitor (not tree)
        monitor->setUpdateCallback([node,
                                    visualizer,
                                    psw_pub,
                                    nbv_pub,
                                    last_octomap_publish,
                                    octomap_publish_rate,
                                    publish_free,
                                    use_moveit,
                                    monitor,
                                    params]()
                                   {
            auto callback_start = node->now();
            RCLCPP_INFO(node->get_logger(), "Semantic occupancy map callback triggered");
            
            auto viz_start = node->now();
            if (visualizer)
            {
                visualizer->publishMarkers(publish_free);
            }
            auto viz_end = node->now();

            auto now = node->now();
            if ((now - *last_octomap_publish).seconds() >= (1.0 / octomap_publish_rate))
            {
                auto publish_start = node->now();
                auto lock_start = node->now();
                monitor->getSemanticMapTree()->lockRead();
                auto lock_end = node->now();
                
                // Publish to MoveIt with standard OcTree format if enabled
                auto moveit_start = node->now();
                if (use_moveit && psw_pub)
                {                    
                    // Use standard OcTree serialization
                    octomap_msgs::msg::Octomap moveit_msg;
                    moveit_msg.header.frame_id = params.map_frame;
                    moveit_msg.header.stamp = now;
                    
                    if (octomap_msgs::binaryMapToMsg(*monitor->getSemanticMapTree(), moveit_msg))
                    {
                        // Convert the id to the standard OcTree id
                        // This works because the serialization only writes the occupancy data
                        // and the tree structure is the same for both types
                        moveit_msg.id = "OcTree";

                        octomap_msgs::msg::OctomapWithPose octomap_with_pose;
                        octomap_with_pose.header = moveit_msg.header;
                        octomap_with_pose.octomap = moveit_msg;
                        octomap_with_pose.origin.orientation.w = 1.0;
                        
                        moveit_msgs::msg::PlanningSceneWorld world_msg;
                        world_msg.octomap = octomap_with_pose;
                        psw_pub->publish(world_msg);
                        
                        RCLCPP_DEBUG(node->get_logger(), "Published standard OcTree to MoveIt (%zu nodes)", 
                                    monitor->getSemanticMapTree()->size());
                    }
                    else
                    {
                        RCLCPP_WARN(node->get_logger(), "Failed to serialize standard tree for MoveIt");
                    }
                }
                auto moveit_end = node->now();
                
                // Publish full semantic tree map to NBV planner
                // Need to publish the full tree since the planner needs all semantic info
                auto nbv_start = node->now();
                octomap_msgs::msg::Octomap octomap_msg;
                octomap_msg.header.frame_id = params.map_frame;
                octomap_msg.header.stamp = now;
                if (octomap_msgs::fullMapToMsg(*monitor->getSemanticMapTree(), octomap_msg))
                {
                    // Save the octomap in a custom message that includes bounding box info
                    erwinia_os_occupancy_map::msg::CustomOctomap nbv_msg;
                    nbv_msg.header = octomap_msg.header;
                    nbv_msg.octomap = octomap_msg;
                    
                    if (params.use_bounding_box)
                    {
                        nbv_msg.has_bounding_box = true;
                        nbv_msg.bbx_min.x = params.bbx_min.x();
                        nbv_msg.bbx_min.y = params.bbx_min.y();
                        nbv_msg.bbx_min.z = params.bbx_min.z();
                        nbv_msg.bbx_max.x = params.bbx_max.x();
                        nbv_msg.bbx_max.y = params.bbx_max.y();
                        nbv_msg.bbx_max.z = params.bbx_max.z();
                    }
                    else
                    {
                        nbv_msg.has_bounding_box = false;
                        nbv_msg.bbx_min.x = nbv_msg.bbx_min.y = nbv_msg.bbx_min.z = 0.0;
                        nbv_msg.bbx_max.x = nbv_msg.bbx_max.y = nbv_msg.bbx_max.z = 0.0;
                    }
                    
                    nbv_pub->publish(nbv_msg);

                    RCLCPP_INFO(node->get_logger(), "Published semantic octomap to NBV planner with %zu nodes", 
                                monitor->getSemanticMapTree()->size());
                }
                else
                {
                    RCLCPP_WARN(node->get_logger(), "Failed to serialize semantic tree for NBV planner");
                }
                auto nbv_end = node->now();
                
                monitor->getSemanticMapTree()->unlockRead();
                *last_octomap_publish = now;
                
                auto publish_end = node->now();
                double lock_time = (lock_end - lock_start).seconds() * 1000.0;
                double moveit_time = (moveit_end - moveit_start).seconds() * 1000.0;
                double nbv_time = (nbv_end - nbv_start).seconds() * 1000.0;
                double publish_time = (publish_end - publish_start).seconds() * 1000.0;
                RCLCPP_INFO(node->get_logger(), "[TIMING] Publish: Lock: %.2fms | MoveIt: %.2fms | NBV: %.2fms | Total: %.2fms",
                           lock_time, moveit_time, nbv_time, publish_time);
            }
            
            auto callback_end = node->now();
            double viz_time = (viz_end - viz_start).seconds() * 1000.0;
            double total_time = (callback_end - callback_start).seconds() * 1000.0;
            RCLCPP_INFO(node->get_logger(), "[TIMING] Update callback: Viz: %.2fms | Total: %.2fms", viz_time, total_time);
            });

        // Start monitoring
        monitor->startMonitor();

        RCLCPP_INFO(logger, "Semantic octomap server ready");
        RCLCPP_INFO(logger, "  Map frame: %s", params.map_frame.c_str());
        RCLCPP_INFO(logger, "  Resolution: %.3f m", params.resolution);
        RCLCPP_INFO(logger, "  Max range: %.2f m", params.max_range);
        RCLCPP_INFO(logger, "  PointCloud topic: %s", pointcloud_topic.c_str());
        RCLCPP_INFO(logger, "  MoveIt integration: %s", use_moveit ? "ENABLED" : "DISABLED");
    }
    else
    {
        // ==================== STANDARD MODE ====================
        // Create pointcloud updater and add to monitor
        auto pc_updater = std::make_shared<PointCloudUpdater>(pointcloud_topic, params);
        pc_updater->initialize(node, tf_buffer);
        monitor->addUpdater(pc_updater);

        RCLCPP_INFO(logger, "Using PointCloudUpdater on: %s", pointcloud_topic.c_str());

        // Create visualizer with OccupancyMapTree (if enabled)
        std::shared_ptr<OccupancyMapVisualizer> visualizer;
        if (enable_viz)
        {
            visualizer = std::make_shared<OccupancyMapVisualizer>(
                node, monitor->getMapTree(), params.map_frame, viz_topic);
            visualizer->setUpdateRate(viz_rate);
        }

        // Octomap publisher for planning scene world updates (only if use_moveit is enabled)
        rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr psw_pub;
        if (use_moveit)
        {
            psw_pub = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
                planning_scene_world_topic, rclcpp::QoS(1).transient_local());
            RCLCPP_INFO(node->get_logger(), "Planning scene world publisher created on topic: %s", planning_scene_world_topic.c_str());
        }

        // Create NBV octomap publisher according to user settings
        rclcpp::QoS nbv_qos = rclcpp::QoS(1); // Default: Reliable + volatile
        if (nbv_octomap_qos_transient_local)
        {
            nbv_qos = rclcpp::QoS(1).transient_local(); // Reliable + transient local
        }
        auto nbv_pub = node->create_publisher<erwinia_os_occupancy_map::msg::CustomOctomap>(
            nbv_octomap_topic, nbv_qos);
        RCLCPP_INFO(node->get_logger(), "NBV octomap publisher created on topic: %s", nbv_octomap_topic.c_str());

        // Last publish time - use shared_ptr so it persists for lambda
        auto last_octomap_publish = std::make_shared<rclcpp::Time>(node->now());

        // Connect update callback
        monitor->setUpdateCallback([node,
                                    visualizer,
                                    psw_pub,
                                    nbv_pub,
                                    last_octomap_publish,
                                    octomap_publish_rate,
                                    publish_free,
                                    use_moveit,
                                    monitor,
                                    params]()
                                   {
            auto callback_start = node->now();
            RCLCPP_INFO(node->get_logger(), "Occupancy map callback triggered");
            
            auto viz_start = node->now();
            if (visualizer)
            {
                visualizer->publishMarkers(publish_free);
                RCLCPP_INFO(node->get_logger(), "Occupancy map visualization updated");
            }
            auto viz_end = node->now();

            auto now = node->now();
            RCLCPP_INFO(node->get_logger(), "Occupancy map node time: %.3f", now.seconds());
            RCLCPP_INFO(node->get_logger(), "Occupancy map last publish time: %.3f", last_octomap_publish->seconds());
            if ((now - *last_octomap_publish).seconds() >= (1.0 / octomap_publish_rate))
            {
                auto publish_start = node->now();
                RCLCPP_INFO(node->get_logger(), "Preparing to publish octomap");

                octomap_msgs::msg::Octomap octomap_msg;
                octomap_msg.header.frame_id = params.map_frame;
                octomap_msg.header.stamp = now;

                auto lock_start = node->now();
                monitor->getMapTree()->lockRead();
                auto lock_end = node->now();
                
                // Convert the octomap to message (binarized)
                if (octomap_msgs::binaryMapToMsg(*monitor->getMapTree(), octomap_msg))
                {
                    // Give the octomap a neutral pose
                    octomap_msgs::msg::OctomapWithPose octomap_with_pose;
                    octomap_with_pose.header = octomap_msg.header;
                    octomap_with_pose.octomap = octomap_msg;
                    octomap_with_pose.origin.orientation.w = 1.0;

                    // Create a planning scene world message and publish (only if use_moveit is enabled)
                    if (use_moveit && psw_pub)
                    {
                        moveit_msgs::msg::PlanningSceneWorld world_msg;
                        world_msg.octomap = octomap_with_pose;
                        psw_pub->publish(world_msg);
                    }

                    // Create custom message with bounding box for NBV planner
                    erwinia_os_occupancy_map::msg::CustomOctomap nbv_msg;
                    nbv_msg.header = octomap_msg.header;
                    nbv_msg.octomap = octomap_msg;
                    
                    // Add bounding box information if configured
                    if (params.use_bounding_box)
                    {
                        nbv_msg.has_bounding_box = true;
                        nbv_msg.bbx_min.x = params.bbx_min.x();
                        nbv_msg.bbx_min.y = params.bbx_min.y();
                        nbv_msg.bbx_min.z = params.bbx_min.z();
                        nbv_msg.bbx_max.x = params.bbx_max.x();
                        nbv_msg.bbx_max.y = params.bbx_max.y();
                        nbv_msg.bbx_max.z = params.bbx_max.z();
                    }
                    else
                    {
                        nbv_msg.has_bounding_box = false;
                        // Initialize to zeros (will be ignored by receiver)
                        nbv_msg.bbx_min.x = nbv_msg.bbx_min.y = nbv_msg.bbx_min.z = 0.0;
                        nbv_msg.bbx_max.x = nbv_msg.bbx_max.y = nbv_msg.bbx_max.z = 0.0;
                    }
                    
                    // Publish custom message to NBV topic
                    nbv_pub->publish(nbv_msg);

                    // Print info
                    RCLCPP_INFO(node->get_logger(), "Published octomap with %zu nodes (bbox=%s)", 
                                monitor->getMapTree()->size(), 
                                params.use_bounding_box ? "enabled" : "disabled");
                }
                
                monitor->getMapTree()->unlockRead();
                *last_octomap_publish = now;
                
                auto publish_end = node->now();
                double lock_time = (lock_end - lock_start).seconds() * 1000.0;
                double publish_time = (publish_end - publish_start).seconds() * 1000.0;
                RCLCPP_INFO(node->get_logger(), "[TIMING] Publish: Lock: %.2fms | Total: %.2fms", lock_time, publish_time);
            }
            
            auto callback_end = node->now();
            double viz_time = (viz_end - viz_start).seconds() * 1000.0;
            double total_time = (callback_end - callback_start).seconds() * 1000.0;
            RCLCPP_INFO(node->get_logger(), "[TIMING] Update callback: Viz: %.2fms | Total: %.2fms", viz_time, total_time);
            });

        // Start monitoring
        monitor->startMonitor();

        RCLCPP_INFO(logger, "Standard octomap server ready");
        RCLCPP_INFO(logger, "  Map frame: %s", params.map_frame.c_str());
        RCLCPP_INFO(logger, "  Resolution: %.3f m", params.resolution);
        RCLCPP_INFO(logger, "  Max range: %.2f m", params.max_range);
        RCLCPP_INFO(logger, "  PointCloud topic: %s", pointcloud_topic.c_str());
        RCLCPP_INFO(logger, "  MoveIt integration: %s", use_moveit ? "ENABLED" : "DISABLED");
    }

    // Clear service: /occupancy_map/clear
    auto clear_service = node->create_service<std_srvs::srv::Trigger>(
        "/occupancy_map/clear",
        [node, monitor, use_semantics](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            RCLCPP_INFO(node->get_logger(), "Clearing occupancy map...");
            if (use_semantics)
            {
                auto tree = monitor->getSemanticMapTree();
                tree->lockWrite();
                tree->clear();
                tree->unlockWrite();
            }
            else
            {
                auto tree = monitor->getMapTree();
                tree->lockWrite();
                tree->clear();
                tree->unlockWrite();
            }
            RCLCPP_INFO(node->get_logger(), "Occupancy map cleared successfully");
            response->success = true;
            response->message = "Occupancy map cleared successfully";
        });
    RCLCPP_INFO(logger, "Clear service available at: /occupancy_map/clear");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
