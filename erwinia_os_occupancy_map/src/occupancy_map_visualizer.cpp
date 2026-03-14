/**
 * @file occupancy_map_visualizer.cpp
 * @brief Implementation of RViz visualization
 */

#include "erwinia_os_occupancy_map/occupancy_map_visualizer.hpp"
#include <geometry_msgs/msg/point.hpp>

namespace erwinia_os_occupancy_map
{

    OccupancyMapVisualizer::OccupancyMapVisualizer(
        const rclcpp::Node::SharedPtr &node,
        const OccupancyMapTreePtr &tree,
        const std::string &map_frame,
        const std::string &topic)
        : node_(node), tree_base_(tree.get()), tree_standard_(tree), map_frame_(map_frame), 
          logger_(node_->get_logger()), last_publish_time_(node_->now()), update_rate_(1.0)
    {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
        RCLCPP_INFO(logger_, "OccupancyMapVisualizer created (standard) on topic: %s", topic.c_str());
    }

    OccupancyMapVisualizer::OccupancyMapVisualizer(
        const rclcpp::Node::SharedPtr &node,
        const SemanticOccupancyMapTreePtr &tree,
        const std::string &map_frame,
        const std::string &topic)
        : node_(node), tree_base_(tree.get()), tree_semantic_(tree), map_frame_(map_frame), 
          logger_(node_->get_logger()), last_publish_time_(node_->now()), update_rate_(1.0)
    {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
        RCLCPP_INFO(logger_, "OccupancyMapVisualizer created (semantic) on topic: %s", topic.c_str());
    }

    void OccupancyMapVisualizer::setUpdateRate(double rate)
    {
        update_rate_ = rate;
    }

    void OccupancyMapVisualizer::publishMarkers(bool publish_free)
    {
        // Rate limiting
        auto now = node_->now();
        if ((now - last_publish_time_).seconds() < 1.0 / update_rate_)
        {
            return;
        }
        last_publish_time_ = now;

        if (!tree_base_)
        {
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        // Lock tree for reading (use appropriate tree pointer)
        if (tree_standard_)
            tree_standard_->lockRead();
        else if (tree_semantic_)
            tree_semantic_->lockRead();

        // Create occupied voxel markers
        double resolution = tree_base_->getResolution();
        
        visualization_msgs::msg::Marker occupied_marker;
        occupied_marker.header.frame_id = map_frame_;
        occupied_marker.header.stamp = now;
        occupied_marker.ns = "occupied_cells";
        occupied_marker.id = 0;
        occupied_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        occupied_marker.action = visualization_msgs::msg::Marker::ADD;
        occupied_marker.scale.x = resolution;
        occupied_marker.scale.y = resolution;
        occupied_marker.scale.z = resolution;
        
        // For standard tree, use red. For semantic, use per-vertex colors
        if (tree_standard_)
        {
            occupied_marker.color.r = 1.0;
            occupied_marker.color.g = 0.0;
            occupied_marker.color.b = 0.0;
            occupied_marker.color.a = 0.8;
        }
        else
        {
            // Semantic tree uses per-vertex colors
            occupied_marker.color.a = 0.8;  // Default alpha
        }

        // Optional: free voxel markers
        visualization_msgs::msg::Marker free_marker;
        if (publish_free)
        {
            free_marker.header.frame_id = map_frame_;
            free_marker.header.stamp = now;
            free_marker.ns = "free_cells";
            free_marker.id = 1;
            free_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            free_marker.action = visualization_msgs::msg::Marker::ADD;
            free_marker.scale.x = resolution;
            free_marker.scale.y = resolution;
            free_marker.scale.z = resolution;
            free_marker.color.r = 0.0;
            free_marker.color.g = 1.0;
            free_marker.color.b = 0.0;
            free_marker.color.a = 0.15;
        }

        // Iterate through octree - use standard or semantic tree appropriately
        if (tree_standard_)
        {
            for (auto it = tree_standard_->begin_leafs(); it != tree_standard_->end_leafs(); ++it)
            {
                if (tree_standard_->isNodeOccupied(*it))
                {
                    geometry_msgs::msg::Point point;
                    point.x = it.getX();
                    point.y = it.getY();
                    point.z = it.getZ();
                    occupied_marker.points.push_back(point);
                }
                else if (publish_free)
                {
                    geometry_msgs::msg::Point point;
                    point.x = it.getX();
                    point.y = it.getY();
                    point.z = it.getZ();
                    free_marker.points.push_back(point);
                }
            }
        }
        else if (tree_semantic_)
        {
            for (auto it = tree_semantic_->begin_leafs(); it != tree_semantic_->end_leafs(); ++it)
            {
                if (tree_semantic_->isNodeOccupied(*it))
                {
                    geometry_msgs::msg::Point point;
                    point.x = it.getX();
                    point.y = it.getY();
                    point.z = it.getZ();
                    occupied_marker.points.push_back(point);
                    
                    // Add per-vertex color based on class_id
                    int32_t class_id = it->getClassId();
                    occupied_marker.colors.push_back(getClassColor(class_id));
                }
                else if (publish_free)
                {
                    geometry_msgs::msg::Point point;
                    point.x = it.getX();
                    point.y = it.getY();
                    point.z = it.getZ();
                    free_marker.points.push_back(point);
                }
            }
        }

        // Unlock tree
        if (tree_standard_)
            tree_standard_->unlockRead();
        else if (tree_semantic_)
            tree_semantic_->unlockRead();

        // Add markers to array
        if (!occupied_marker.points.empty())
        {
            marker_array.markers.push_back(occupied_marker);
        }

        if (publish_free && !free_marker.points.empty())
        {
            marker_array.markers.push_back(free_marker);
        }

        // Add bounding box visualization if enabled
        bool has_bbx = (tree_standard_ && tree_standard_->bbxSet()) || 
                       (tree_semantic_ && tree_semantic_->bbxSet());
        if (has_bbx)
        {
            publishBoundingBox(marker_array, now);
        }

        // Publish
        marker_pub_->publish(marker_array);

        RCLCPP_INFO(logger_, "Published occupancy map visualization with %zu occupied voxels and %zu free voxels",
                      occupied_marker.points.size(),
                      publish_free ? free_marker.points.size() : 0);
    }

    void OccupancyMapVisualizer::publishBoundingBox(
        visualization_msgs::msg::MarkerArray &marker_array,
        const rclcpp::Time &stamp,
        const std::optional<octomap::point3d> &bbx_min_opt,
        const std::optional<octomap::point3d> &bbx_max_opt)
    {
        // Get bounding box min/max - use provided values or fall back to tree's values
        octomap::point3d bbx_min, bbx_max;
        if (bbx_min_opt.has_value())
            bbx_min = bbx_min_opt.value();
        else if (tree_standard_)
            bbx_min = tree_standard_->getBBXMin();
        else if (tree_semantic_)
            bbx_min = tree_semantic_->getBBXMin();
            
        if (bbx_max_opt.has_value())
            bbx_max = bbx_max_opt.value();
        else if (tree_standard_)
            bbx_max = tree_standard_->getBBXMax();
        else if (tree_semantic_)
            bbx_max = tree_semantic_->getBBXMax();

        // Create LINE_LIST marker for bounding box edges
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header.frame_id = map_frame_;
        bbox_marker.header.stamp = stamp;
        bbox_marker.ns = "bounding_box";
        bbox_marker.id = 100;
        bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;
        bbox_marker.scale.x = 0.02; // Line width
        bbox_marker.color.r = 0.0;
        bbox_marker.color.g = 0.5;
        bbox_marker.color.b = 1.0;
        bbox_marker.color.a = 1.0;
        bbox_marker.pose.orientation.w = 1.0;

        // Define 8 corners of the bounding box
        std::array<geometry_msgs::msg::Point, 8> corners;

        // Bottom face (z = min)
        corners[0].x = bbx_min.x();
        corners[0].y = bbx_min.y();
        corners[0].z = bbx_min.z();
        corners[1].x = bbx_max.x();
        corners[1].y = bbx_min.y();
        corners[1].z = bbx_min.z();
        corners[2].x = bbx_max.x();
        corners[2].y = bbx_max.y();
        corners[2].z = bbx_min.z();
        corners[3].x = bbx_min.x();
        corners[3].y = bbx_max.y();
        corners[3].z = bbx_min.z();

        // Top face (z = max)
        corners[4].x = bbx_min.x();
        corners[4].y = bbx_min.y();
        corners[4].z = bbx_max.z();
        corners[5].x = bbx_max.x();
        corners[5].y = bbx_min.y();
        corners[5].z = bbx_max.z();
        corners[6].x = bbx_max.x();
        corners[6].y = bbx_max.y();
        corners[6].z = bbx_max.z();
        corners[7].x = bbx_min.x();
        corners[7].y = bbx_max.y();
        corners[7].z = bbx_max.z();

        // Bottom face edges (4 edges)
        bbox_marker.points.push_back(corners[0]);
        bbox_marker.points.push_back(corners[1]);
        bbox_marker.points.push_back(corners[1]);
        bbox_marker.points.push_back(corners[2]);
        bbox_marker.points.push_back(corners[2]);
        bbox_marker.points.push_back(corners[3]);
        bbox_marker.points.push_back(corners[3]);
        bbox_marker.points.push_back(corners[0]);

        // Top face edges (4 edges)
        bbox_marker.points.push_back(corners[4]);
        bbox_marker.points.push_back(corners[5]);
        bbox_marker.points.push_back(corners[5]);
        bbox_marker.points.push_back(corners[6]);
        bbox_marker.points.push_back(corners[6]);
        bbox_marker.points.push_back(corners[7]);
        bbox_marker.points.push_back(corners[7]);
        bbox_marker.points.push_back(corners[4]);

        // Vertical edges connecting top and bottom (4 edges)
        bbox_marker.points.push_back(corners[0]);
        bbox_marker.points.push_back(corners[4]);
        bbox_marker.points.push_back(corners[1]);
        bbox_marker.points.push_back(corners[5]);
        bbox_marker.points.push_back(corners[2]);
        bbox_marker.points.push_back(corners[6]);
        bbox_marker.points.push_back(corners[3]);
        bbox_marker.points.push_back(corners[7]);

        marker_array.markers.push_back(bbox_marker);

        RCLCPP_DEBUG(logger_, "Published bounding box visualization");
    }

    std_msgs::msg::ColorRGBA OccupancyMapVisualizer::getClassColor(int32_t class_id) const
    {
        std_msgs::msg::ColorRGBA color;
        color.a = 0.8;

        // Handle negative class IDs (background)
        if (class_id < 0)
        {
            color.r = 128.0 / 255.0;
            color.g = 128.0 / 255.0;
            color.b = 128.0 / 255.0;
            return color;
        }

        // Predefined color palette for classes 0-19
        // const uint8_t palette[][3] = {
        //     {230, 25, 75},    // Red - class 0
        //     {60, 180, 75},    // Green - class 1
        //     {255, 225, 25},   // Yellow - class 2
        //     {0, 130, 200},    // Blue - class 3
        //     {245, 130, 48},   // Orange - class 4
        //     {145, 30, 180},   // Purple - class 5
        //     {70, 240, 240},   // Cyan - class 6
        //     {240, 50, 230},   // Magenta - class 7
        //     {210, 245, 60},   // Lime - class 8
        //     {250, 190, 212},  // Pink - class 9
        //     {0, 128, 128},    // Teal - class 10
        //     {220, 190, 255},  // Lavender - class 11
        //     {170, 110, 40},   // Brown - class 12
        //     {255, 250, 200},  // Beige - class 13
        //     {128, 0, 0},      // Maroon - class 14
        //     {170, 255, 195},  // Mint - class 15
        //     {128, 128, 0},    // Olive - class 16
        //     {255, 215, 180},  // Coral - class 17
        //     {0, 0, 128},      // Navy - class 18
        //     {128, 128, 128}   // Grey - class 19
        // };

        const uint8_t palette[][3] = {
            {0, 150, 255},    // Blue - class 0
            {230, 25, 75},    // Red - class 1
            {60, 180, 75},    // Green - class 2
            {255, 225, 25},   // Yellow - class 3
            {245, 130, 48},   // Orange - class 4
            {145, 30, 180},   // Purple - class 5
            {70, 240, 240},   // Cyan - class 6
            {240, 50, 230},   // Magenta - class 7
            {210, 245, 60},   // Lime - class 8
            {250, 190, 212},  // Pink - class 9
            {0, 128, 128},    // Teal - class 10
            {220, 190, 255},  // Lavender - class 11
            {170, 110, 40},   // Brown - class 12
            {255, 250, 200},  // Beige - class 13
            {128, 0, 0},      // Maroon - class 14
            {170, 255, 195},  // Mint - class 15
            {128, 128, 0},    // Olive - class 16
            {255, 215, 180},  // Coral - class 17
            {0, 0, 128},      // Navy - class 18
            {128, 128, 128}   // Grey - class 19
        };

        if (class_id < 20)
        {
            color.r = palette[class_id][0] / 255.0;
            color.g = palette[class_id][1] / 255.0;
            color.b = palette[class_id][2] / 255.0;
            return color;
        }

        // For class IDs >= 20, use hash function with brightness adjustment
        uint32_t hash = static_cast<uint32_t>(class_id) * 2654435761u;  // Knuth's multiplicative hash
        uint8_t r = (hash >> 16) & 0xFF;
        uint8_t g = (hash >> 8) & 0xFF;
        uint8_t b = hash & 0xFF;

        // Ensure colors are not too dark
        r = (r < 50) ? r + 100 : r;
        g = (g < 50) ? g + 100 : g;
        b = (b < 50) ? b + 100 : b;

        color.r = r / 255.0;
        color.g = g / 255.0;
        color.b = b / 255.0;

        return color;
    }

} // namespace erwinia_os_occupancy_map