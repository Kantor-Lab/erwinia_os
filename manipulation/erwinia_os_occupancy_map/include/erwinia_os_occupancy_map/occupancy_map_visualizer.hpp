/**
 * @file occupancy_map_visualizer.hpp
 * @brief RViz visualization for occupancy map
 */

#pragma once

#include "erwinia_os_occupancy_map/occupancy_map_tree.hpp"
#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <octomap/OccupancyOcTreeBase.h>
#include <memory>
#include <string>
#include <optional>

namespace erwinia_os_occupancy_map
{

    /**
     * @brief Publishes RViz markers for occupancy map visualization
     *
     * Creates marker arrays showing occupied voxels (and optionally free voxels)
     * for debugging and visualization purposes.
     * 
     * Works with any OccupancyOcTreeBase-derived tree (OcTree, SemanticOcTree, etc.)
     */
    class OccupancyMapVisualizer
    {
    public:
        /**
         * @brief Constructor for standard OccupancyMapTree
         */
        OccupancyMapVisualizer(
            const rclcpp::Node::SharedPtr &node,
            const OccupancyMapTreePtr &tree,
            const std::string &map_frame,
            const std::string &topic = "occupancy_map_markers");

        /**
         * @brief Constructor for SemanticOccupancyMapTree
         */
        OccupancyMapVisualizer(
            const rclcpp::Node::SharedPtr &node,
            const SemanticOccupancyMapTreePtr &tree,
            const std::string &map_frame,
            const std::string &topic = "occupancy_map_markers");

        /**
         * @brief Publish marker visualization
         * @param publish_free Whether to publish free voxels (can be slow)
         */
        void publishMarkers(bool publish_free = false);

        /**
         * @brief Set update rate (Hz)
         */
        void setUpdateRate(double rate);

    private:
        /**
         * @brief Publish bounding box marker
         * @param marker_array Marker array to add bbox marker to
         * @param stamp Timestamp for the marker
         * @param bbx_min Optional minimum corner of bounding box (uses tree's if not provided)
         * @param bbx_max Optional maximum corner of bounding box (uses tree's if not provided)
         */
        void publishBoundingBox(
            visualization_msgs::msg::MarkerArray &marker_array,
            const rclcpp::Time &stamp,
            const std::optional<octomap::point3d> &bbx_min = std::nullopt,
            const std::optional<octomap::point3d> &bbx_max = std::nullopt);

        /**
         * @brief Get color for semantic class ID
         */
        std_msgs::msg::ColorRGBA getClassColor(int32_t class_id) const;

        rclcpp::Node::SharedPtr node_;
        octomap::AbstractOcTree *tree_base_;  // Polymorphic base pointer
        OccupancyMapTreePtr tree_standard_;   // Keep alive standard tree
        SemanticOccupancyMapTreePtr tree_semantic_;  // Keep alive semantic tree
        std::string map_frame_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Logger logger_;
        rclcpp::Time last_publish_time_;
        double update_rate_;
    };

    using OccupancyMapVisualizerPtr = std::shared_ptr<OccupancyMapVisualizer>;

} // namespace erwinia_os_occupancy_map
