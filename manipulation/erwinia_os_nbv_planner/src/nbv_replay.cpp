/**
 * @file nbv_replay.cpp
 * @brief Next-Best-View replay for manipulation workspace
 * 
 * Need to launch:
 * - ros2 bag play <bag_file> --clock --topics /tf /firefly_left/image_raw /firefly_left/camera_info /firefly_right/image_raw /firefly_right/camera_info
 * - ros2 launch erwinia_os_nbv_planner nbv_demo.launch.py use_sim_time:=true use_gazebo:=false planner_type:=replay stereo_matcher_model_trt:=fs_224x448_vit-small_iters5.plan detection_model_trt:=yolo26_large_seg_rivendale_v6_fold1.plan export_viewpoint_voxels:=true metrics_dir:=metrics/penn_state run:=tree_11/baseline/ gt_points_file:=metrics/penn_state/tree_11/gt_points.json
 */

//  ros2 run rmw_zenoh_cpp rmw_zenohd
//  ros2 bag play /media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_eval_recordings/recording_baseline_1/ --clock --topics /tf /firefly_left/image_raw /firefly_left/camera_info /firefly_right/image_raw /firefly_right/camera_info
//  ros2 launch erwinia_os_description rosbag_playback.launch.py
//  ros2 launch erwinia_os_nbv_planner nbv_demo.launch.py use_sim_time:=true use_gazebo:=false planner_type:=replay octomap_resolution:=0.02 camera_scaled_width:=896 camera_scaled_height:=672 stereo_matcher_model_trt:=fs_672x896_vit-small_iters16.plan detection_model_trt:=yolo26_large_seg_rivendale_v6_fold1.plan semantic_mismatch_penalty:=0.2 semantic_confidence_boost:=0.3 export_viewpoint_voxels:=true metrics_dir:=metrics/penn_state run:=tree_11/baseline/ gt_points_file:=metrics/penn_state/tree_11/gt_points.json conf_thresh:=0.15


#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <queue>
#include <algorithm>
#include <optional>
#include <thread>
#include <fstream>
#include <mutex>
#include <cstring>
#include <cmath>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

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
#include "erwinia_os_nbv_planner/ground_truth_evaluator.hpp"
#include "erwinia_os_nbv_planner/nbv_planner_utils.hpp"

using namespace erwinia_os_nbv_planner;
using namespace erwinia_os_nbv_planner::conversions;

// ---------------------------------------------------------------------------
// CloudAccumulatorNode
// Subscribes to a PointCloud2 topic, transforms every cloud into the map frame,
// strips all fields except x/y/z/rgb, and incrementally overwrites a binary
// PCD file so the accumulated cloud is always up-to-date on disk.
// ---------------------------------------------------------------------------
class CloudAccumulatorNode : public rclcpp::Node
{
public:
    CloudAccumulatorNode(const std::string &cloud_topic,
                         const std::string &map_frame,
                         const std::string &output_path)
        : Node("cloud_accumulator_node"),
          map_frame_(map_frame),
          output_path_(output_path)
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic,
            rclcpp::QoS(50).best_effort(),
            std::bind(&CloudAccumulatorNode::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "CloudAccumulatorNode: topic='%s'  output='%s'",
                    cloud_topic.c_str(), output_path_.c_str());
    }

    // Call this explicitly after the replay loop finishes to flush the final state.
    void savePCD()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (n_accumulated_ == 0)
            return;

        std::ofstream f(output_path_, std::ios::binary);
        if (!f.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Cannot open PCD file for writing: %s", output_path_.c_str());
            return;
        }
        f << "# .PCD v0.7\n"
          << "VERSION 0.7\n"
          << "FIELDS x y z rgb\n"
          << "SIZE 4 4 4 4\n"
          << "TYPE F F F F\n"
          << "COUNT 1 1 1 1\n"
          << "WIDTH "    << n_accumulated_ << "\n"
          << "HEIGHT 1\n"
          << "VIEWPOINT 0 0 0 1 0 0 0\n"
          << "POINTS " << n_accumulated_ << "\n"
          << "DATA binary\n";
        f.write(reinterpret_cast<const char *>(accumulated_data_.data()),
                static_cast<std::streamsize>(accumulated_data_.size()));
        RCLCPP_INFO(get_logger(), "Saved accumulated cloud: %u points -> %s",
                    n_accumulated_, output_path_.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Locate x/y/z/rgb field byte offsets in the incoming cloud
        int x_off = -1, y_off = -1, z_off = -1, rgb_off = -1;
        for (const auto &field : msg->fields)
        {
            if      (field.name == "x")   x_off   = static_cast<int>(field.offset);
            else if (field.name == "y")   y_off   = static_cast<int>(field.offset);
            else if (field.name == "z")   z_off   = static_cast<int>(field.offset);
            else if (field.name == "rgb") rgb_off = static_cast<int>(field.offset);
        }
        if (x_off < 0 || y_off < 0 || z_off < 0 || rgb_off < 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Incoming cloud is missing x/y/z/rgb fields, skipping");
            return;
        }

        // Look up transform from cloud frame to map frame
        geometry_msgs::msg::TransformStamped tf_stamped;
        try
        {
            tf_stamped = tf_buffer_->lookupTransform(
                map_frame_, msg->header.frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Cloud TF lookup failed: %s", ex.what());
            return;
        }

        // Build rotation matrix from quaternion
        const auto &rot = tf_stamped.transform.rotation;
        const auto &tr  = tf_stamped.transform.translation;
        double qx = rot.x, qy = rot.y, qz = rot.z, qw = rot.w;
        double tx = tr.x,  ty = tr.y,  tz = tr.z;
        double R[3][3] = {
            {1.0 - 2.0*(qy*qy + qz*qz),       2.0*(qx*qy - qw*qz),       2.0*(qx*qz + qw*qy)},
            {      2.0*(qx*qy + qw*qz), 1.0 - 2.0*(qx*qx + qz*qz),       2.0*(qy*qz - qw*qx)},
            {      2.0*(qx*qz - qw*qy),       2.0*(qy*qz + qw*qx), 1.0 - 2.0*(qx*qx + qy*qy)}
        };

        // Transform points and pack into 16-byte x/y/z/rgb layout
        const uint8_t  *in_ptr  = msg->data.data();
        const uint32_t  in_step = msg->point_step;
        const uint32_t  n_pts   = msg->width * msg->height;

        std::vector<uint8_t> batch;
        batch.reserve(n_pts * 16);
        uint32_t added = 0;

        for (uint32_t i = 0; i < n_pts; ++i)
        {
            const uint8_t *p = in_ptr + i * in_step;
            float xi, yi, zi;
            std::memcpy(&xi, p + x_off, 4);
            std::memcpy(&yi, p + y_off, 4);
            std::memcpy(&zi, p + z_off, 4);
            if (!std::isfinite(xi) || !std::isfinite(yi) || !std::isfinite(zi))
                continue;

            float xo = static_cast<float>(R[0][0]*xi + R[0][1]*yi + R[0][2]*zi + tx);
            float yo = static_cast<float>(R[1][0]*xi + R[1][1]*yi + R[1][2]*zi + ty);
            float zo = static_cast<float>(R[2][0]*xi + R[2][1]*yi + R[2][2]*zi + tz);

            float rgb;
            std::memcpy(&rgb, p + rgb_off, 4);

            uint8_t pt[16];
            std::memcpy(pt +  0, &xo,  4);
            std::memcpy(pt +  4, &yo,  4);
            std::memcpy(pt +  8, &zo,  4);
            std::memcpy(pt + 12, &rgb, 4);
            batch.insert(batch.end(), pt, pt + 16);
            ++added;
        }

        if (added == 0)
            return;

        // Append to accumulated buffer under the lock
        {
            std::lock_guard<std::mutex> lock(mutex_);
            accumulated_data_.insert(accumulated_data_.end(), batch.begin(), batch.end());
            n_accumulated_ += added;
        }

        // Overwrite the PCD file so the latest state is always on disk
        savePCD();
    }

    std::string map_frame_;
    std::string output_path_;
    std::shared_ptr<tf2_ros::Buffer>                               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>                    tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    std::mutex           mutex_;
    std::vector<uint8_t> accumulated_data_;
    uint32_t             n_accumulated_{0};
};

static int run(std::shared_ptr<rclcpp::Node> node,
               rclcpp::executors::SingleThreadedExecutor &executor)
{
    // Service client to clear the occupancy map
    auto clear_client = node->create_client<std_srvs::srv::Trigger>("/occupancy_map/clear");

    // Wait for sim time if needed
    if (node->get_parameter("use_sim_time").as_bool())
        waitForSimClock(node);

    auto config = loadConfiguration(node);
    printConfiguration(config, node->get_logger());

    // Create and register cloud accumulator node (only if save_accumulated_cloud is true)
    static constexpr bool kSaveAccumulatedCloud = false;
    static constexpr const char *kCloudSaveTopic = "/firefly_left/points2";

    std::shared_ptr<CloudAccumulatorNode> accumulator;
    if (kSaveAccumulatedCloud)
    {
        std::string pcd_path = config.metrics_data_dir + "/accumulated_cloud.pcd";
        accumulator = std::make_shared<CloudAccumulatorNode>(kCloudSaveTopic, config.map_frame, pcd_path);
        executor.add_node(accumulator);
    }

    const bool gt_file_requested = !config.gt_points_file.empty();

    // Initialize visualizer for planner visuals or GT marker publication
    std::shared_ptr<NBVVisualizer> visualizer;
    if (config.visualize || gt_file_requested)
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);

    // Initialize TF2 buffer and listener
    RCLCPP_DEBUG(node->get_logger(), "\nInitializing TF2 Buffer and Listener");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize octomap interface
    RCLCPP_DEBUG(node->get_logger(), "\nInitializing OctoMap Interface");
    auto octomap_interface = std::make_shared<OctoMapInterface>(node, config.octomap_topic, true);

    // Clear occupancy map before starting
    if (!callClearMap(node, clear_client, node->get_logger()))
    {
        rclcpp::shutdown();
        return 1;
    }

    // Clear previous visualization markers
    if (visualizer)
    {
        visualizer->clearAllMarkers();
    }

    // Initialize evaluation if enabled
    std::vector<ViewpointEvaluation> all_metrics;
    double initial_time = 0.0;
    int viewpoint_index = 0;
    GroundTruthEvaluator evaluator(node);
    const std::string evaluation_json_path = config.metrics_data_dir + "/evaluation_metrics.json";
    bool do_evaluation = false;

    auto get_bbox = [&]() -> std::optional<std::pair<octomap::point3d, octomap::point3d>>
    {
        octomap::point3d min_out;
        octomap::point3d max_out;
        if (octomap_interface->getBoundingBox(min_out, max_out))
        {
            return std::make_pair(min_out, max_out);
        }
        return std::nullopt;
    };

    auto publish_semantic_markers = [&](const std::vector<Cluster> &latest_clusters)
    {
        if (!visualizer)
            return;
        if (config.visualize)
        {
            visualizer->publishClusteredVoxels(
                latest_clusters,
                octomap_interface->getResolution(),
                false,
                0.55f,
                "semantic_clusters",
                octomap_interface->getOctomapFrameId());
        }
        if (evaluator.hasGroundTruth())
        {
            visualizer->publishSemanticPoints(
                evaluator.getGroundTruthInFrame(octomap_interface->getOctomapFrameId()),
                octomap_interface->getResolution() * 1.8,
                0.95f,
                true,
                "ground_truth_segments",
                octomap_interface->getOctomapFrameId());
        }
    };

    auto build_evaluation = [&](int current_viewpoint_index, const geometry_msgs::msg::Pose *camera_pose, double elapsed_time) -> ViewpointEvaluation
    {
        auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
        auto [occupied, free] = octomap_interface->getVoxelCounts();
        std::optional<geometry_msgs::msg::Pose> optional_pose;
        if (camera_pose != nullptr)
        {
            optional_pose = *camera_pose;
        }
        return evaluator.buildViewpointEvaluation(
            current_viewpoint_index,
            elapsed_time,
            optional_pose,
            latest_clusters,
            octomap_interface->getOctomapFrameId(),
            get_bbox(),
            static_cast<int>(occupied),
            static_cast<int>(free),
            octomap_interface->calculateCoverage());
    };

    if (gt_file_requested)
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for first octomap message before initializing GT evaluation...");
        rclcpp::Rate wait_rate(10);
        while (rclcpp::ok() && !octomap_interface->isTreeAvailable())
        {
            wait_rate.sleep();
        }
    }

    if (gt_file_requested && octomap_interface->isSemanticTree())
    {
        do_evaluation = true;
        if (!evaluator.loadGroundTruthFile(config.gt_points_file))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to load ground truth file: %s", config.gt_points_file.c_str());
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Ground truth loaded successfully");
        evaluator.setPointTransform(
            Eigen::Vector3d(config.gt_points_position[0], config.gt_points_position[1], config.gt_points_position[2]),
            Eigen::Quaterniond(
                Eigen::AngleAxisd(config.gt_points_rotation[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(config.gt_points_rotation[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(config.gt_points_rotation[0], Eigen::Vector3d::UnitX())));

        // Store initial time for relative time calculations
        initial_time = node->now().seconds();

        auto initial_evaluation = build_evaluation(viewpoint_index, nullptr, 0.0);
        all_metrics.push_back(initial_evaluation);
        evaluator.logViewpointEvaluation(initial_evaluation);
        publish_semantic_markers(octomap_interface->clusterSemanticVoxels(false));
        evaluator.writeEvaluationsToJson(all_metrics, evaluation_json_path);
        viewpoint_index = 1;
    }
    else if (gt_file_requested)
    {
        RCLCPP_WARN(
            node->get_logger(),
            "GT file was provided, but the first received octomap is '%s'. Skipping GT evaluation.",
            octomap_interface->getTreeTypeString().c_str());
    }

    // Motion accumulator — tracks total path length (translation + rotation) between
    // octomap updates so that moves-and-returns are counted as motion.
    struct MotionAccumulator {
        double translation = 0.0;
        double rotation    = 0.0;
        geometry_msgs::msg::TransformStamped last_tf;
        bool has_last_tf = false;
    };
    MotionAccumulator motion_acc;
    std::mutex motion_mutex;

    auto motion_timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [&]() {
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer->lookupTransform(
                    config.map_frame, config.camera_optical_link, tf2::TimePointZero);
            } catch (const tf2::TransformException &) { return; }

            std::lock_guard<std::mutex> lock(motion_mutex);
            if (motion_acc.has_last_tf) {
                const auto &t0 = motion_acc.last_tf.transform.translation;
                const auto &t1 = tf.transform.translation;
                double dt = std::sqrt(std::pow(t1.x - t0.x, 2) +
                                      std::pow(t1.y - t0.y, 2) +
                                      std::pow(t1.z - t0.z, 2));

                const auto &q0 = motion_acc.last_tf.transform.rotation;
                const auto &q1 = tf.transform.rotation;
                double dot = std::min(1.0, std::abs(q0.x*q1.x + q0.y*q1.y +
                                                    q0.z*q1.z + q0.w*q1.w));
                double dtheta = 2.0 * std::acos(dot);

                motion_acc.translation += dt;
                motion_acc.rotation    += dtheta;
            }
            motion_acc.last_tf     = tf;
            motion_acc.has_last_tf = true;
        });

    // Main NBV Replay Loop
    int i = 0;
    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "\n********** NBV Replay Octomap Update %d **********", i);

        // Wait for the next octomap update
        rclcpp::Time prev_update_time = octomap_interface->getLastUpdateTime();
        rclcpp::Rate spin_rate(10);
        while (rclcpp::ok())
        {
            if (octomap_interface->getLastUpdateTime().nanoseconds() != prev_update_time.nanoseconds())
                break;
            spin_rate.sleep();
        }
        if (!rclcpp::ok())
            break;

        // Capture timestamp immediately after update is detected, before any slow processing
        double update_time = node->now().seconds() - initial_time;

        // Consume accumulated motion since last octomap update and reset
        double total_translation, total_rotation;
        {
            std::lock_guard<std::mutex> lock(motion_mutex);
            total_translation = motion_acc.translation;
            total_rotation    = motion_acc.rotation;
            motion_acc.translation = 0.0;
            motion_acc.rotation    = 0.0;
        }
        bool eef_moved = (total_translation > 1e-3 || total_rotation > 1e-2);

        // Look up current camera pose for evaluation metadata
        geometry_msgs::msg::Pose camera_pose;
        bool has_camera_pose = false;
        try
        {
            auto current_tf = tf_buffer->lookupTransform(
                config.map_frame, config.camera_optical_link, tf2::TimePointZero);
            camera_pose.position.x  = current_tf.transform.translation.x;
            camera_pose.position.y  = current_tf.transform.translation.y;
            camera_pose.position.z  = current_tf.transform.translation.z;
            camera_pose.orientation = current_tf.transform.rotation;
            has_camera_pose = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
        }

        const auto latest_clusters = octomap_interface->isSemanticTree()
            ? octomap_interface->clusterSemanticVoxels(false)
            : std::vector<Cluster>{};

        if (do_evaluation)
        {
            const int current_viewpoint_index = eef_moved ? viewpoint_index++ : std::max(0, viewpoint_index - 1);
            auto evaluation = build_evaluation(current_viewpoint_index, has_camera_pose ? &camera_pose : nullptr, update_time);
            if (!eef_moved && !all_metrics.empty())
            {
                all_metrics.back() = evaluation;
            }
            else
            {
                all_metrics.push_back(evaluation);
                if (config.export_viewpoint_voxels)
                {
                    evaluator.exportVoxelSnapshot(
                        *octomap_interface,
                        current_viewpoint_index,
                        config.viewpoint_voxel_export_dir);
                }
            }
            evaluator.logViewpointEvaluation(evaluation);
            publish_semantic_markers(latest_clusters);
            evaluator.writeEvaluationsToJson(all_metrics, evaluation_json_path);
        }
        else if (visualizer && octomap_interface->isSemanticTree())
        {
            publish_semantic_markers(latest_clusters);
        }

        if (visualizer)
        {
            visualizer->clearAllMarkers();
            if (octomap_interface->isSemanticTree())
            {
                publish_semantic_markers(latest_clusters);
            }
        }

        ++i;
    }

    // Clear visualization
    if (visualizer)
    {
        visualizer->clearAllMarkers();
    }

    // Flush final accumulated cloud to disk
    if (accumulator)
        accumulator->savePCD();

    // Check if there was an error in the loop
    if (!rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "\nNBV planning interrupted by shutdown signal, exiting...");
        rclcpp::shutdown();
        return 1;
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
    std::thread spinner([&executor]()
                        { executor.spin(); });

    int result = run(node, executor);

    rclcpp::shutdown();
    spinner.join();
    return result;
}
