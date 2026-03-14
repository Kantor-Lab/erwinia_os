#include "erwinia_os_occupancy_map/semantic_pointcloud_updater.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace erwinia_os_occupancy_map
{

    SemanticPointCloudUpdater::SemanticPointCloudUpdater(
        const std::string &point_cloud_topic,
        const OccupancyMapParameters &params)
        : OccupancyMapUpdater("semantic_pointcloud"),
          point_cloud_topic_(point_cloud_topic),
          params_(params),
          logger_(rclcpp::get_logger("semantic_pointcloud_updater"))
    {
    }

    bool SemanticPointCloudUpdater::initialize(
        const rclcpp::Node::SharedPtr &node,
        const std::shared_ptr<tf2_ros::Buffer> &tf_buffer)
    {
        node_ = node;
        tf_buffer_ = tf_buffer;
        logger_ = node_->get_logger();

        // Declare semantic-specific parameters
        node_->declare_parameter("semantic_min_occupancy", 0.5);
        semantic_min_occupancy_ = node_->get_parameter("semantic_min_occupancy").as_double();

        RCLCPP_INFO(logger_, "Initialized SemanticPointCloudUpdater");
        RCLCPP_INFO(logger_, "  Topic: %s", point_cloud_topic_.c_str());
        RCLCPP_INFO(logger_, "  Semantic min occupancy: %.2f", semantic_min_occupancy_);

        return true;
    }

    void SemanticPointCloudUpdater::start()
    {
        if (active_)
        {
            RCLCPP_WARN(logger_, "SemanticPointCloudUpdater already active");
            return;
        }

        if (!semantic_tree_)
        {
            RCLCPP_ERROR(logger_, "Semantic tree not set! Call setSemanticTree() first.");
            return;
        }

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic_,
            rclcpp::QoS(1).best_effort(),
            std::bind(&SemanticPointCloudUpdater::pointCloudCallback, this, std::placeholders::_1));

        active_ = true;
        RCLCPP_INFO(logger_, "Started SemanticPointCloudUpdater");
    }

    void SemanticPointCloudUpdater::stop()
    {
        if (!active_)
            return;

        cloud_sub_.reset();
        active_ = false;
        RCLCPP_INFO(logger_, "Stopped SemanticPointCloudUpdater");
    }

    void SemanticPointCloudUpdater::setSemanticTree(const std::shared_ptr<SemanticOccupancyMapTree> &tree)
    {
        semantic_tree_ = tree;
    }

    bool SemanticPointCloudUpdater::parseSemanticPointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        std::vector<PointXYZRGBSemanticConfidence> &points)
    {
        // Check for required fields
        bool has_x = false, has_y = false, has_z = false;
        bool has_rgb = false, has_class_id = false, has_confidence = false;

        for (const auto &field : msg->fields)
        {
            if (field.name == "x") has_x = true;
            else if (field.name == "y") has_y = true;
            else if (field.name == "z") has_z = true;
            else if (field.name == "rgb") has_rgb = true;
            else if (field.name == "class_id") has_class_id = true;
            else if (field.name == "confidence") has_confidence = true;
        }

        if (!has_x || !has_y || !has_z || !has_class_id || !has_confidence)
        {
            RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 5000,
                                  "Semantic point cloud missing required fields: x,y,z,class_id,confidence");
            return false;
        }

        points.clear();
        points.reserve(msg->width * msg->height);

        // Create iterators
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<int32_t> iter_class(*msg, "class_id");
        sensor_msgs::PointCloud2ConstIterator<float> iter_conf(*msg, "confidence");

        // RGB is optional
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_rgb;
        if (has_rgb)
        {
            iter_rgb = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "rgb");
        }

        for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_class, ++iter_conf)
        {
            PointXYZRGBSemanticConfidence point;
            point.x = *iter_x;
            point.y = *iter_y;
            point.z = *iter_z;
            point.class_id = *iter_class;
            point.confidence = *iter_conf;
            
            if (iter_rgb)
            {
                point.rgb = **iter_rgb;
                ++(*iter_rgb);
            }
            else
            {
                point.rgb = 0.0f;
            }

            // Skip invalid points
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            if (point.confidence <= 0.0f)
                continue;

            points.push_back(point);
        }

        return true;
    }

    void SemanticPointCloudUpdater::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto callback_start = node_->now();
        
        if (!semantic_tree_)
        {
            RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 5000, "Semantic tree not set!");
            return;
        }

        // Parse semantic point cloud
        auto parse_start = node_->now();
        std::vector<PointXYZRGBSemanticConfidence> points;
        if (!parseSemanticPointCloud(msg, points))
        {
            return;
        }
        auto parse_end = node_->now();

        if (points.empty())
        {
            RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000, "Empty semantic point cloud");
            return;
        }

        // Get sensor origin in map frame
        auto transform_start = node_->now();
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                params_.map_frame,
                msg->header.frame_id,
                msg->header.stamp,
                rclcpp::Duration::from_seconds(0.1));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000,
                                 "TF lookup failed: %s", ex.what());
            return;
        }
        auto transform_end = node_->now();

        octomap::point3d sensor_origin(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        // Step 1: Aggregate by voxel key with max-confidence fusion
        auto aggregate_start = node_->now();
        std::unordered_map<octomap::OcTreeKey, SemanticBest, OcTreeKeyHash, OcTreeKeyEqual> best;
        best.reserve(points.size());

        for (const auto &p : points)
        {
            if (p.confidence <= 0.0f)
                continue;

            // Transform point to map frame
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header = msg->header;
            point_in.point.x = p.x;
            point_in.point.y = p.y;
            point_in.point.z = p.z;

            tf2::doTransform(point_in, point_out, transform);

            octomap::point3d point_map(point_out.point.x, point_out.point.y, point_out.point.z);

            // // Range filtering
            // if (!isInRange(point_map, sensor_origin))
            //     continue;

            // // Ground plane filtering
            // if (params_.filter_ground_plane && isGroundPlane(point_map))
            //     continue;

            // Get voxel key
            octomap::OcTreeKey key;
            if (!semantic_tree_->coordToKeyChecked(point_map, key))
                continue;

            // Just keep the label with highest confidence for this voxel
            auto &v = best[key];
            if (p.confidence > v.confidence)
            {
                v.confidence = p.confidence;
                v.class_id = p.class_id;
                v.rgb = p.rgb;
            }
        }

        if (best.empty())
        {
            RCLCPP_DEBUG(logger_, "No valid points after filtering");
            return;
        }
        auto aggregate_end = node_->now();

        // Step 2: Build octomap::Pointcloud from unique voxel keys
        auto build_cloud_start = node_->now();
        octomap::Pointcloud octomap_cloud;
        octomap_cloud.reserve(best.size());
        for (const auto &kv : best)
        {
            octomap_cloud.push_back(semantic_tree_->keyToCoord(kv.first));
        }
        auto build_cloud_end = node_->now();

        // Step 3: Insert point cloud for occupancy (thread-safe) with lazy eval
        auto insertion_start = node_->now();
        {
            auto write_lock = semantic_tree_->writing();
            semantic_tree_->insertPointCloud(octomap_cloud, sensor_origin, params_.max_range, true, true);
        }
        auto insertion_end = node_->now();

        // Step 4: Integrate semantics per-key with occupancy gating
        auto semantic_integration_start = node_->now();
        {
            auto write_lock = semantic_tree_->writing();
            size_t integrated_count = 0;

            for (const auto &kv : best)
            {
                const auto &key = kv.first;
                const auto &v = kv.second;

                // Only assign semantics if voxel is occupied enough
                auto *node = semantic_tree_->search(key);
                if (!node)
                    continue;
                if (node->getOccupancy() < semantic_min_occupancy_) // Not occupied enough
                    continue;

                semantic_tree_->integrateNodeSemantic(key, v.class_id, v.confidence, true, true,
                                                     params_.semantic_confidence_boost,
                                                     params_.semantic_mismatch_penalty);
                integrated_count++;
            }

            // Update inner nodes once after all updates (occupancy + semantics)
            semantic_tree_->updateInnerOccupancySemantic();

            RCLCPP_DEBUG(logger_, "Integrated semantics for %zu/%zu voxels", integrated_count, best.size());
        }
        auto semantic_integration_end = node_->now();

        // Trigger update callback
        semantic_tree_->triggerUpdateCallback();

        auto callback_end = node_->now();
        
        // Timing summary
        double parse_time = (parse_end - parse_start).seconds() * 1000.0;
        double transform_time = (transform_end - transform_start).seconds() * 1000.0;
        double aggregate_time = (aggregate_end - aggregate_start).seconds() * 1000.0;
        double build_time = (build_cloud_end - build_cloud_start).seconds() * 1000.0;
        double insertion_time = (insertion_end - insertion_start).seconds() * 1000.0;
        double semantic_time = (semantic_integration_end - semantic_integration_start).seconds() * 1000.0;
        double total_time = (callback_end - callback_start).seconds() * 1000.0;
        
        RCLCPP_INFO(logger_, "[TIMING] Semantic PC: %zu pts->%zu voxels | Parse: %.2fms | TF: %.2fms | Aggregate: %.2fms | Build: %.2fms | Insert: %.2fms | Semantic: %.2fms | Total: %.2fms",
                    points.size(), best.size(), parse_time, transform_time, aggregate_time, 
                    build_time, insertion_time, semantic_time, total_time);
    }

    bool SemanticPointCloudUpdater::isInRange(const octomap::point3d &point, const octomap::point3d &origin) const
    {
        double distance = (point - origin).norm();
        if (distance < params_.min_range || distance > params_.max_range)
        {
            return false;
        }

        // Bounding box check
        if (params_.use_bounding_box)
        {
            if (point.x() < params_.bbx_min.x() || point.x() > params_.bbx_max.x() ||
                point.y() < params_.bbx_min.y() || point.y() > params_.bbx_max.y() ||
                point.z() < params_.bbx_min.z() || point.z() > params_.bbx_max.z())
            {
                return false;
            }
        }

        return true;
    }

    // bool SemanticPointCloudUpdater::isGroundPlane(const octomap::point3d &point) const
    // {
    //     return std::abs(point.z()) < params_.ground_distance_threshold;
    // }

} // namespace erwinia_os_occupancy_map
