/**
 * @file pointcloud_updater.hpp
 * @brief PointCloud2 processor for octree updates
 */

#pragma once

#include "erwinia_os_occupancy_map/occupancy_map_updater.hpp"
#include "erwinia_os_occupancy_map/occupancy_map_monitor.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace erwinia_os_occupancy_map
{

    /**
     * @brief Updates octree from PointCloud2 messages
     *
     * Subscribes to point cloud topic, transforms points to map frame,
     * and performs ray casting to update free and occupied space.
     */
    class PointCloudUpdater : public OccupancyMapUpdater
    {
    public:
        /**
         * @brief Constructor
         * @param point_cloud_topic Topic name for PointCloud2
         * @param params Map parameters
         */
        PointCloudUpdater(
            const std::string &point_cloud_topic,
            const OccupancyMapParameters &params);

        /**
         * @brief Destructor
         */
        virtual ~PointCloudUpdater() = default;

        /**
         * @brief Initialize with ROS node and TF
         */
        bool initialize(
            const rclcpp::Node::SharedPtr &node,
            const std::shared_ptr<tf2_ros::Buffer> &tf_buffer) override;

        /**
         * @brief Start subscribing to point cloud
         */
        void start() override;

        /**
         * @brief Stop subscribing
         */
        void stop() override;

    private:
        /**
         * @brief Callback for point cloud messages
         */
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /**
         * @brief Check if point is within valid range
         */
        bool isInRange(const octomap::point3d &point, const octomap::point3d &origin) const;

        /**
         * @brief Filter ground plane (optional)
         */
        // bool isGroundPlane(const octomap::point3d &point) const;

        std::string topic_;                                                  ///< Point cloud topic
        OccupancyMapParameters params_;                                      ///< Configuration
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_; ///< Subscriber
        bool active_;                                                        ///< Processing flag
        size_t points_processed_;                                            ///< Statistics
    };

} // namespace erwinia_os_occupancy_map
