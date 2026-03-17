/**
 * @file occupancy_map_updater.hpp
 * @brief Base class for sensor-specific octree updaters
 */

#pragma once

#include "erwinia_os_occupancy_map/occupancy_map_tree.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>

namespace erwinia_os_occupancy_map
{

    // Forward declaration
    class OccupancyMapMonitor;

    /**
     * @brief Abstract base class for octree updaters
     *
     * Different sensor types (pointcloud, depth image, etc.) implement
     * this interface to update the occupancy map.
     */
    class OccupancyMapUpdater
    {
    public:
        /**
         * @brief Constructor
         * @param type Type identifier (e.g., "PointCloudUpdater")
         */
        explicit OccupancyMapUpdater(const std::string &type);

        /**
         * @brief Virtual destructor
         */
        virtual ~OccupancyMapUpdater();

        /**
         * @brief Initialize updater with ROS node and TF buffer
         * @param node ROS2 node
         * @param tf_buffer TF2 buffer for transforms
         * @return true if initialization successful
         */
        virtual bool initialize(
            const rclcpp::Node::SharedPtr &node,
            const std::shared_ptr<tf2_ros::Buffer> &tf_buffer) = 0;

        /**
         * @brief Start processing sensor data
         */
        virtual void start() = 0;

        /**
         * @brief Stop processing sensor data
         */
        virtual void stop() = 0;

        /**
         * @brief Get type identifier
         */
        std::string getType() const { return type_; }

        /**
         * @brief Set the monitor that owns this updater
         * @param monitor Parent monitor
         */
        void setMonitor(OccupancyMapMonitor *monitor);

    protected:
        std::string type_;                           ///< Updater type identifier
        OccupancyMapMonitor *monitor_;               ///< Parent monitor
        OccupancyMapTreePtr tree_;                   ///< Shared octree
        rclcpp::Node::SharedPtr node_;               ///< ROS node
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_; ///< TF buffer
        rclcpp::Logger logger_;                      ///< Logger
    };

    using OccupancyMapUpdaterPtr = std::shared_ptr<OccupancyMapUpdater>;

} // namespace erwinia_os_occupancy_map
