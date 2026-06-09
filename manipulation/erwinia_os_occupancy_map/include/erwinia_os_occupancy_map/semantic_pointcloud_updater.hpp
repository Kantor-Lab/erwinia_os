#pragma once

#include "erwinia_os_occupancy_map/occupancy_map_updater.hpp"
#include "erwinia_os_occupancy_map/occupancy_map_monitor.hpp"
#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <map>

namespace erwinia_os_occupancy_map
{

    /**
     * @brief Custom point type for semantic point clouds
     */
    struct PointXYZRGBSemanticConfidence
    {
        float x;            // 3D position X
        float y;            // 3D position Y
        float z;            // 3D position Z
        float rgb;          // Packed RGB color (PCL/RViz compatible)
        int32_t class_id;   // Semantic class identifier
        float confidence;   // Detection confidence [0.0, 1.0]
    };

    /**
     * @brief Hash function for OcTreeKey
     */
    struct OcTreeKeyHash
    {
        size_t operator()(const octomap::OcTreeKey &k) const noexcept
        {
            return (static_cast<size_t>(k.k[0]) * 73856093u) ^
                   (static_cast<size_t>(k.k[1]) * 19349663u) ^
                   (static_cast<size_t>(k.k[2]) * 83492791u);
        }
    };

    /**
     * @brief Equality function for OcTreeKey
     */
    struct OcTreeKeyEqual
    {
        bool operator()(const octomap::OcTreeKey &a, const octomap::OcTreeKey &b) const noexcept
        {
            return a.k[0] == b.k[0] && a.k[1] == b.k[1] && a.k[2] == b.k[2];
        }
    };

    /**
     * @brief Strict-weak-ordering comparator for OcTreeKey (needed for std::map/std::set)
     */
    struct OcTreeKeyCompare
    {
        bool operator()(const octomap::OcTreeKey &a, const octomap::OcTreeKey &b) const noexcept
        {
            if (a.k[0] != b.k[0]) return a.k[0] < b.k[0];
            if (a.k[1] != b.k[1]) return a.k[1] < b.k[1];
            return a.k[2] < b.k[2];
        }
    };

    /**
     * @brief Best semantic information per voxel
     */
    struct SemanticBest
    {
        int32_t class_id = 0;
        float confidence = 0.0f;
        float rgb = 0.0f; // Optional: keep color
    };

    /**
     * @brief Updates semantic octree from semantic PointCloud2 messages
     *
     * Expected point format: x, y, z, rgb, class_id, confidence
     * Aggregates by voxel with max-confidence fusion strategy.
     */
    class SemanticPointCloudUpdater : public OccupancyMapUpdater
    {
    public:
        /**
         * @brief Constructor
         * @param point_cloud_topic Topic name for semantic PointCloud2
         * @param params Map parameters
         */
        SemanticPointCloudUpdater(
            const std::string &point_cloud_topic,
            const OccupancyMapParameters &params);

        virtual ~SemanticPointCloudUpdater() = default;

        /**
         * @brief Initialize with ROS node and TF
         */
        bool initialize(
            const rclcpp::Node::SharedPtr &node,
            const std::shared_ptr<tf2_ros::Buffer> &tf_buffer) override;

        /**
         * @brief Start subscribing to semantic point cloud
         */
        void start() override;

        /**
         * @brief Stop subscribing
         */
        void stop() override;

        /**
         * @brief Set the semantic tree reference
         */
        void setSemanticTree(const std::shared_ptr<SemanticOccupancyMapTree> &tree);

    private:
        /**
         * @brief Callback for semantic point cloud messages
         */
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /**
         * @brief Parse semantic point cloud from PointCloud2 message
         */
        bool parseSemanticPointCloud(
            const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
            std::vector<PointXYZRGBSemanticConfidence> &points);

        /**
         * @brief Check if point is within valid range
         */
        bool isInRange(const octomap::point3d &point, const octomap::point3d &origin) const;

        /**
         * @brief Filter ground plane (optional)
         */
        // bool isGroundPlane(const octomap::point3d &point) const;

        // ROS resources
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Logger logger_;

        // Subscription
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

        // Configuration
        std::string point_cloud_topic_;
        OccupancyMapParameters params_;

        // Semantic tree reference
        std::shared_ptr<SemanticOccupancyMapTree> semantic_tree_;

        // State
        bool active_{false};

        // Minimum occupancy threshold for semantic integration
        double semantic_min_occupancy_{0.5};
    };

} // namespace erwinia_os_occupancy_map
