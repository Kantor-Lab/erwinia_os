/**
 * @file nbv_visualizer.cpp
 * @brief Implementation of RViz visualization for NBV planning
 */

#include "erwinia_os_nbv_planner/nbv_visualizer.hpp"

// #include <matplot/matplot.h>
#include <iomanip>
#include <algorithm>

namespace erwinia_os_nbv_planner
{

    NBVVisualizer::NBVVisualizer(
        const rclcpp::Node::SharedPtr &node,
        const std::string &map_frame,
        const std::string &topic)
        : node_(node),
          map_frame_(map_frame),
          logger_(node_->get_logger())
    {
        // Create marker publisher with default QoS (VOLATILE) to match RViz expectations
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            topic, rclcpp::QoS(10));

        RCLCPP_INFO(logger_, "NBVVisualizer created for frame '%s', publishing to '%s'",
                    map_frame_.c_str(), topic.c_str());
    }

    std_msgs::msg::ColorRGBA NBVVisualizer::colorForLabel(int label, float alpha)
    {
        std_msgs::msg::ColorRGBA color;
        color.a = alpha;

        // Handle negative class IDs (background)
        if (label < 0)
        {
            color.r = 128.0 / 255.0;
            color.g = 128.0 / 255.0;
            color.b = 128.0 / 255.0;
            return color;
        }

        // Match the semantic octomap voxel palette from OccupancyMapVisualizer.
        const uint8_t palette[][3] = {
            {0, 150, 255},   // Blue - class 0
            {230, 25, 75},   // Red - class 1
            {60, 180, 75},   // Green - class 2
            {255, 225, 25},  // Yellow - class 3
            {245, 130, 48},  // Orange - class 4
            {145, 30, 180},  // Purple - class 5
            {70, 240, 240},  // Cyan - class 6
            {240, 50, 230},  // Magenta - class 7
            {210, 245, 60},  // Lime - class 8
            {250, 190, 212}, // Pink - class 9
            {0, 128, 128},   // Teal - class 10
            {220, 190, 255}, // Lavender - class 11
            {170, 110, 40},  // Brown - class 12
            {255, 250, 200}, // Beige - class 13
            {128, 0, 0},     // Maroon - class 14
            {170, 255, 195}, // Mint - class 15
            {128, 128, 0},   // Olive - class 16
            {255, 215, 180}, // Coral - class 17
            {0, 0, 128},     // Navy - class 18
            {128, 128, 128}  // Grey - class 19
        };

        if (label < 20)
        {
            color.r = palette[label][0] / 255.0;
            color.g = palette[label][1] / 255.0;
            color.b = palette[label][2] / 255.0;
            return color;
        }

        // For class IDs >= 20, use hash function with brightness adjustment
        uint32_t hash = static_cast<uint32_t>(label) * 2654435761u; // Knuth's multiplicative hash
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

    void NBVVisualizer::publishVoxels(
        const std::vector<octomap::point3d> &voxels,
        double voxel_size,
        const std_msgs::msg::ColorRGBA &color,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (voxels.empty())
        {
            return;
        }

        // Create vector of same color for all voxels
        std_msgs::msg::ColorRGBA actual_color = color;
        actual_color.a = alpha;
        std::vector<std_msgs::msg::ColorRGBA> colors(voxels.size(), actual_color);

        // Call the more general function
        publishVoxels(voxels, voxel_size, colors, alpha, ns, frame_id);
    }

    void NBVVisualizer::publishVoxels(
        const std::vector<octomap::point3d> &voxels,
        double voxel_size,
        const std::vector<std_msgs::msg::ColorRGBA> &colors,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (colors.size() != voxels.size())
        {
            RCLCPP_ERROR(logger_, "publishVoxels: colors vector size (%zu) must match voxels size (%zu)",
                         colors.size(), voxels.size());
            return;
        }

        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = viz_frame;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        if (voxels.empty())
        {
            marker_pub_->publish(marker_array);
            return;
        }

        // Create individual CUBE markers for each voxel (to support per-voxel colors)
        for (size_t i = 0; i < voxels.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = viz_frame;
            marker.header.stamp = now;
            marker.ns = ns;
            marker.id = static_cast<int>(i + 1);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = voxels[i].x();
            marker.pose.position.y = voxels[i].y();
            marker.pose.position.z = voxels[i].z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = voxel_size;
            marker.color = colors[i];
            marker.color.a = alpha; // Apply alpha override
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published %zu voxels with individual colors", voxels.size());
    }

    void NBVVisualizer::publishClusteredVoxels(
        const std::vector<Cluster> &clusters,
        double voxel_size,
        bool plot_centers,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (clusters.empty())
        {
            return;
        }

        // Generate colors based on class_id (if available) or label
        std::vector<std_msgs::msg::ColorRGBA> colors;
        colors.reserve(clusters.size());
        for (const auto &cluster : clusters)
        {
            int color_index = (cluster.class_id >= 0) ? cluster.class_id : cluster.label;
            colors.push_back(colorForLabel(color_index, alpha));
        }

        // Call the more general function
        publishClusteredVoxels(clusters, voxel_size, colors, plot_centers, alpha, ns, frame_id);
    }

    void NBVVisualizer::publishClusteredVoxels(
        const std::vector<Cluster> &clusters,
        double voxel_size,
        const std_msgs::msg::ColorRGBA &color,
        bool plot_centers,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (clusters.empty())
        {
            return;
        }

        // Create vector of same color for all clusters
        std_msgs::msg::ColorRGBA actual_color = color;
        actual_color.a = alpha;
        std::vector<std_msgs::msg::ColorRGBA> colors(clusters.size(), actual_color);

        // Call the more general function
        publishClusteredVoxels(clusters, voxel_size, colors, plot_centers, alpha, ns, frame_id);
    }

    void NBVVisualizer::publishClusteredVoxels(
        const std::vector<Cluster> &clusters,
        double voxel_size,
        const std::vector<std_msgs::msg::ColorRGBA> &colors,
        bool plot_centers,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (colors.size() != clusters.size())
        {
            RCLCPP_ERROR(logger_, "publishClusteredVoxels: colors vector size (%zu) must match clusters size (%zu)",
                         colors.size(), clusters.size());
            return;
        }

        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = viz_frame;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        if (plot_centers)
        {
            visualization_msgs::msg::Marker del_centers;
            del_centers.header.frame_id = viz_frame;
            del_centers.header.stamp = now;
            del_centers.ns = ns + "_centers";
            del_centers.id = 0;
            del_centers.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(del_centers);
        }

        if (clusters.empty())
        {
            marker_pub_->publish(marker_array);
            return;
        }

        // Create one CUBE_LIST marker per cluster
        int id = 1;
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const auto &cluster = clusters[i];
            if (cluster.points.empty())
                continue;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = viz_frame;
            marker.header.stamp = now;
            marker.ns = ns;
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = voxel_size;
            marker.color = colors[i]; // Use per-cluster color
            marker.color.a = alpha;   // Apply alpha override

            for (const auto &point : cluster.points)
            {
                geometry_msgs::msg::Point p;
                p.x = point.x();
                p.y = point.y();
                p.z = point.z();
                marker.points.push_back(p);
            }

            marker_array.markers.push_back(marker);
        }

        // Add cluster centers as small spheres
        if (plot_centers)
        {
            visualization_msgs::msg::Marker centers;
            centers.header.frame_id = viz_frame;
            centers.header.stamp = now;
            centers.ns = ns + "_centers";
            centers.id = 1;
            centers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            centers.action = visualization_msgs::msg::Marker::ADD;
            centers.pose.orientation.w = 1.0;
            centers.scale.x = centers.scale.y = centers.scale.z = 0.1;
            centers.color.r = 1.0f;
            centers.color.g = 1.0f;
            centers.color.b = 1.0f;
            centers.color.a = 1.0f;

            for (const auto &cluster : clusters)
            {
                if (!cluster.points.empty())
                {
                    geometry_msgs::msg::Point p;
                    p.x = cluster.center.x();
                    p.y = cluster.center.y();
                    p.z = cluster.center.z();
                    centers.points.push_back(p);
                }
            }

            marker_array.markers.push_back(centers);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published %zu clusters with individual colors", clusters.size());
    }

    visualization_msgs::msg::Marker NBVVisualizer::createAxisTriad(
        const geometry_msgs::msg::Pose &pose,
        int id,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        // Note: This creates a LINE_LIST marker for compatibility with single-marker return
        // For proper cylinders, use the publishCoordinates function which creates multiple markers
        visualization_msgs::msg::Marker triad;
        triad.header.frame_id = map_frame_;
        triad.header.stamp = node_->now();
        triad.ns = ns;
        triad.id = id;
        triad.type = visualization_msgs::msg::Marker::LINE_LIST;
        triad.action = visualization_msgs::msg::Marker::ADD;
        triad.pose = pose;
        triad.scale.x = axis_radius * 2.0; // line width (thicker for visibility)

        // Define axis lines in local frame
        geometry_msgs::msg::Point origin, x_end, y_end, z_end;
        origin.x = origin.y = origin.z = 0.0;
        x_end.x = axis_length;
        x_end.y = x_end.z = 0.0;
        y_end.y = axis_length;
        y_end.x = y_end.z = 0.0;
        z_end.z = axis_length;
        z_end.x = z_end.y = 0.0;

        // X axis (red)
        triad.points.push_back(origin);
        triad.points.push_back(x_end);
        // Y axis (green)
        triad.points.push_back(origin);
        triad.points.push_back(y_end);
        // Z axis (blue)
        triad.points.push_back(origin);
        triad.points.push_back(z_end);

        // Colors per vertex
        triad.colors.resize(6);
        // X axis red
        triad.colors[0].r = 1.0f;
        triad.colors[0].a = alpha;
        triad.colors[1].r = 1.0f;
        triad.colors[1].a = alpha;
        // Y axis green
        triad.colors[2].g = 1.0f;
        triad.colors[2].a = alpha;
        triad.colors[3].g = 1.0f;
        triad.colors[3].a = alpha;
        // Z axis blue
        triad.colors[4].b = 1.0f;
        triad.colors[4].a = alpha;
        triad.colors[5].b = 1.0f;
        triad.colors[5].a = alpha;

        return triad;
    }

    std::vector<visualization_msgs::msg::Marker> NBVVisualizer::createAxisTriadCylinders(
        const geometry_msgs::msg::Pose &pose,
        int base_id,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        std::vector<visualization_msgs::msg::Marker> markers;

        // Helper lambda to create a cylinder for one axis
        auto createAxisCylinder = [&](int axis_id, double dx, double dy, double dz, float r, float g, float b)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = node_->now();
            marker.ns = ns;
            marker.id = base_id * 3 + axis_id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Transform the pose to position the cylinder along the axis
            // Cylinder is centered at its pose, oriented along Z axis by default
            // We need to translate it by half the length and rotate it

            // Convert pose quaternion to transformation matrix
            Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                                 pose.orientation.y, pose.orientation.z);
            Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);

            // Direction vector for this axis
            Eigen::Vector3d axis_dir(dx, dy, dz);
            axis_dir.normalize();

            // Position at half the axis length
            Eigen::Vector3d axis_pos = pos + q * (axis_dir * axis_length * 0.5);

            // Create rotation to align Z axis with the desired axis direction
            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Quaterniond axis_rotation;

            // If axis is along +X
            if (std::abs(dx - 1.0) < 0.01)
            {
                axis_rotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 1, 0));
            }
            // If axis is along +Y
            else if (std::abs(dy - 1.0) < 0.01)
            {
                axis_rotation = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1, 0, 0));
            }
            // If axis is along +Z
            else
            {
                axis_rotation = Eigen::Quaterniond::Identity();
            }

            // Combine rotations
            Eigen::Quaterniond final_q = q * axis_rotation;

            marker.pose.position.x = axis_pos.x();
            marker.pose.position.y = axis_pos.y();
            marker.pose.position.z = axis_pos.z();
            marker.pose.orientation.x = final_q.x();
            marker.pose.orientation.y = final_q.y();
            marker.pose.orientation.z = final_q.z();
            marker.pose.orientation.w = final_q.w();

            marker.scale.x = axis_radius * 2.0; // diameter
            marker.scale.y = axis_radius * 2.0; // diameter
            marker.scale.z = axis_length;       // height

            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = alpha;

            return marker;
        };

        // Create X axis (red)
        markers.push_back(createAxisCylinder(0, 1, 0, 0, 1.0f, 0.0f, 0.0f));
        // Create Y axis (green)
        markers.push_back(createAxisCylinder(1, 0, 1, 0, 0.0f, 1.0f, 0.0f));
        // Create Z axis (blue)
        markers.push_back(createAxisCylinder(2, 0, 0, 1, 0.0f, 0.0f, 1.0f));

        return markers;
    }

    void NBVVisualizer::publishCoordinates(
        const std::vector<geometry_msgs::msg::Pose> &poses,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // // Delete previous markers
        // visualization_msgs::msg::Marker del_marker;
        // del_marker.header.frame_id = viz_frame;
        // del_marker.header.stamp = now;
        // del_marker.ns = ns;
        // del_marker.id = 0;
        // del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        // marker_array.markers.push_back(del_marker);

        if (poses.empty())
        {
            marker_pub_->publish(marker_array);
            return;
        }

        // Create axis triad cylinders for each pose
        for (size_t i = 0; i < poses.size(); ++i)
        {
            auto cylinders = createAxisTriadCylinders(
                poses[i],
                static_cast<int>(i + 1),
                axis_length,
                axis_radius,
                alpha,
                ns);

            // Add all three cylinder markers (X, Y, Z axes) with correct frame
            for (auto &marker : cylinders)
            {
                marker.header.frame_id = viz_frame;
                marker.header.stamp = now;
                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published %zu candidate views", poses.size());
    }

    void NBVVisualizer::publishCoordinate(
        const geometry_msgs::msg::Pose &pose,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        // Single pose - wrap in vector and call multi-pose version
        std::vector<geometry_msgs::msg::Pose> poses = {pose};
        publishCoordinates(poses, axis_length, axis_radius, alpha, ns, frame_id);
    }

    void NBVVisualizer::publishCoordinate(
        const Eigen::Vector3d &position,
        const std::array<double, 4> &orientation,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        // Convert Eigen types to geometry_msgs::Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation.x = orientation[0];
        pose.orientation.y = orientation[1];
        pose.orientation.z = orientation[2];
        pose.orientation.w = orientation[3];

        publishCoordinate(pose, axis_length, axis_radius, alpha, ns, frame_id);
    }

    void NBVVisualizer::publishViewpoint(
        const Viewpoint &viewpoint,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        // Convert Viewpoint to geometry_msgs::Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = viewpoint.position.x();
        pose.position.y = viewpoint.position.y();
        pose.position.z = viewpoint.position.z();
        pose.orientation.x = viewpoint.orientation[0];
        pose.orientation.y = viewpoint.orientation[1];
        pose.orientation.z = viewpoint.orientation[2];
        pose.orientation.w = viewpoint.orientation[3];

        // Publish using existing publishCoordinate function
        publishCoordinate(pose, axis_length, axis_radius, alpha, ns, frame_id);
    }

    visualization_msgs::msg::Marker NBVVisualizer::createWireframeSphere(
        const octomap::point3d &center,
        double radius,
        double line_width,
        const std_msgs::msg::ColorRGBA &color,
        const std::string &ns)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = line_width;
        marker.color = color;

        // Create wireframe sphere using latitude/longitude lines
        const int lat_segments = 12;
        const int lon_segments = 16;

        // Latitude circles
        for (int i = 0; i <= lat_segments; ++i)
        {
            double lat = M_PI * (double(i) / lat_segments - 0.5);
            double y = radius * std::sin(lat);
            double r = radius * std::cos(lat);

            for (int j = 0; j < lon_segments; ++j)
            {
                double lon1 = 2.0 * M_PI * double(j) / lon_segments;
                double lon2 = 2.0 * M_PI * double(j + 1) / lon_segments;

                geometry_msgs::msg::Point p1, p2;
                p1.x = r * std::cos(lon1);
                p1.y = y;
                p1.z = r * std::sin(lon1);

                p2.x = r * std::cos(lon2);
                p2.y = y;
                p2.z = r * std::sin(lon2);

                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }

        // Longitude lines
        for (int j = 0; j < lon_segments; ++j)
        {
            double lon = 2.0 * M_PI * double(j) / lon_segments;

            for (int i = 0; i < lat_segments; ++i)
            {
                double lat1 = M_PI * (double(i) / lat_segments - 0.5);
                double lat2 = M_PI * (double(i + 1) / lat_segments - 0.5);

                geometry_msgs::msg::Point p1, p2;
                p1.x = radius * std::cos(lat1) * std::cos(lon);
                p1.y = radius * std::sin(lat1);
                p1.z = radius * std::cos(lat1) * std::sin(lon);

                p2.x = radius * std::cos(lat2) * std::cos(lon);
                p2.y = radius * std::sin(lat2);
                p2.z = radius * std::cos(lat2) * std::sin(lon);

                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }

        return marker;
    }

    void NBVVisualizer::publishTargetRegion(
        const octomap::point3d &center,
        double radius,
        double line_width,
        const std::string &ns,
        const std::string &frame_id)
    {
        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous marker
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = viz_frame;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        // Create wireframe sphere
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f;
        color.g = 0.5f;
        color.b = 0.0f;
        color.a = 0.8f;

        auto sphere = createWireframeSphere(center, radius, line_width, color, ns);
        sphere.header.frame_id = viz_frame;
        sphere.header.stamp = now;
        marker_array.markers.push_back(sphere);

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published target region");
    }

    void NBVVisualizer::publishBoundingBox(
        const octomap::point3d &bbx_min,
        const octomap::point3d &bbx_max,
        const std::string &ns,
        double line_width,
        const std_msgs::msg::ColorRGBA &color,
        const std::string &frame_id)
    {
        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = viz_frame;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        // Create LINE_LIST marker for bounding box edges
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header.frame_id = viz_frame;
        bbox_marker.header.stamp = now;
        bbox_marker.ns = ns;
        bbox_marker.id = 1;
        bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;
        bbox_marker.scale.x = line_width; // Line width
        bbox_marker.pose.orientation.w = 1.0;

        // Set color (use provided color or default to cyan)
        if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
        {
            bbox_marker.color.r = 0.0;
            bbox_marker.color.g = 0.5;
            bbox_marker.color.b = 1.0;
            bbox_marker.color.a = 1.0;
        }
        else
        {
            bbox_marker.color = color;
        }

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
        marker_pub_->publish(marker_array);

        RCLCPP_DEBUG(logger_, "Published bounding box visualization");
    }

    void NBVVisualizer::publishPlane(
        const std::vector<Eigen::Vector3d> &corners,
        const std::string &ns,
        double line_width,
        const std_msgs::msg::ColorRGBA &color,
        const std::string &frame_id)
    {
        if (corners.size() < 3)
        {
            RCLCPP_ERROR(logger_, "publishPlane: need at least 3 corners to define a plane");
            return;
        }

        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = viz_frame;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        // Create LINE_LIST marker for plane edges and grid
        visualization_msgs::msg::Marker plane_marker;
        plane_marker.header.frame_id = viz_frame;
        plane_marker.header.stamp = now;
        plane_marker.ns = ns;
        plane_marker.id = 1;
        plane_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        plane_marker.action = visualization_msgs::msg::Marker::ADD;
        plane_marker.scale.x = line_width; // Line width
        plane_marker.pose.orientation.w = 1.0;

        // Set color (use provided color or default to cyan)
        if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
        {
            plane_marker.color.r = 0.0;
            plane_marker.color.g = 1.0;
            plane_marker.color.b = 1.0;
            plane_marker.color.a = 0.8;
        }
        else
        {
            plane_marker.color = color;
        }

        // Convert corners to geometry_msgs::Point
        std::vector<geometry_msgs::msg::Point> points;
        points.reserve(corners.size());
        for (const auto &corner : corners)
        {
            geometry_msgs::msg::Point p = conversions::eigenToPoint(corner);
            points.push_back(p);
        }

        // Draw plane border (connect all corners in sequence, then close the loop)
        for (size_t i = 0; i < points.size(); ++i)
        {
            plane_marker.points.push_back(points[i]);
            plane_marker.points.push_back(points[(i + 1) % points.size()]);
        }

        marker_array.markers.push_back(plane_marker);
        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published plane visualization with %zu corners", points.size());
    }

    void NBVVisualizer::clearAll(const std::string &ns_suffix)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = ns_suffix.empty() ? "" : ns_suffix;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Cleared all markers in namespace '%s'", ns_suffix.c_str());
    }

    void NBVVisualizer::clearVoxels()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "voxels";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearClusteredVoxels()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "clustered_voxels";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearClusterCenters()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "cluster_centers";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        del_marker.ns = "cluster_centers_labels";
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearCandidateViews()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "candidate_views";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearBestView()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "best_view";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearTargetRegion()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "target_region";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearAllMarkers()
    {
        // Clear all known namespaces
        clearVoxels();
        clearClusteredVoxels();
        clearClusterCenters();
        clearCandidateViews();
        clearBestView();
        clearTargetRegion();
    }

    void NBVVisualizer::publishPoint(
        const octomap::point3d &point,
        int id,
        double size,
        const std_msgs::msg::ColorRGBA &color,
        float alpha,
        const std::string &ns,
        const std::string &text_label,
        const std::string &frame_id)
    {
        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        visualization_msgs::msg::MarkerArray marker_array;

        // Create point marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = viz_frame;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = point.x();
        marker.pose.position.y = point.y();
        marker.pose.position.z = point.z();
        marker.pose.orientation.w = 1.0;

        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        // Use provided color or default to green
        if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = alpha;
        }
        else
        {
            marker.color = color;
            marker.color.a = alpha;
        }

        marker_array.markers.push_back(marker);

        // Add text label if provided
        if (!text_label.empty())
        {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = viz_frame;
            text_marker.header.stamp = node_->now();
            text_marker.ns = ns + "_text";
            text_marker.id = id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = point.x();
            text_marker.pose.position.y = point.y();
            text_marker.pose.position.z = point.z() + size; // Position text above the point
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.05; // Text height
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;
            text_marker.text = text_label;
            marker_array.markers.push_back(text_marker);
        }

        // Publish marker array
        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::publishPoints(
        const std::vector<octomap::point3d> &points,
        double size,
        const std_msgs::msg::ColorRGBA &color,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (points.empty())
        {
            return;
        }

        // Use provided color or default to green
        std_msgs::msg::ColorRGBA actual_color = color;
        if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
        {
            actual_color.r = 0.0f;
            actual_color.g = 1.0f;
            actual_color.b = 0.0f;
            actual_color.a = alpha;
        }
        else
        {
            actual_color.a = alpha;
        }

        // Create vector of same color for all points
        std::vector<std_msgs::msg::ColorRGBA> colors(points.size(), actual_color);
        std::vector<std::string> labels; // Empty labels

        // Call the more general function
        publishPoints(points, colors, labels, size, alpha, ns, frame_id);
    }

    void NBVVisualizer::publishPoints(
        const std::vector<octomap::point3d> &points,
        const std::vector<std_msgs::msg::ColorRGBA> &colors,
        const std::vector<std::string> &labels,
        double size,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (points.empty())
        {
            return;
        }

        // Use frame_id as the visualization frame (default to map_frame_ if empty)
        std::string viz_frame = frame_id.empty() ? map_frame_ : frame_id;

        if (colors.size() != points.size())
        {
            RCLCPP_ERROR(logger_, "publishPoints: colors vector size (%zu) must match points size (%zu)",
                         colors.size(), points.size());
            return;
        }

        if (!labels.empty() && labels.size() != points.size())
        {
            RCLCPP_ERROR(logger_, "publishPoints: labels vector size (%zu) must match points size (%zu) or be empty",
                         labels.size(), points.size());
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        auto stamp = node_->now();

        // Delete all previous markers in this namespace (sphere and text) before
        // publishing new ones. Without this, a GT point that changes category
        // between calls (e.g., correct → missed) retains a stale color because its
        // sequential marker ID changes — the old marker just stays in RViz.
        for (const auto &clear_ns : {ns, ns + "_text"})
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = viz_frame;
            del.header.stamp = stamp;
            del.ns = clear_ns;
            del.id = 0;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(del);
        }

        // Create individual sphere markers for each point (to support per-point colors)
        for (size_t i = 0; i < points.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = viz_frame;
            marker.header.stamp = stamp;
            marker.ns = ns;
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = points[i].x();
            marker.pose.position.y = points[i].y();
            marker.pose.position.z = points[i].z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = size;
            marker.scale.y = size;
            marker.scale.z = size;
            marker.color = colors[i];
            marker.color.a = alpha; // Apply alpha override
            marker_array.markers.push_back(marker);

            // Add text label if provided
            if (!labels.empty() && !labels[i].empty())
            {
                visualization_msgs::msg::Marker text_marker;
                text_marker.header.frame_id = viz_frame;
                text_marker.header.stamp = stamp;
                text_marker.ns = ns + "_text";
                text_marker.id = static_cast<int>(i);
                text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::msg::Marker::ADD;
                text_marker.pose.position.x = points[i].x();
                text_marker.pose.position.y = points[i].y();
                text_marker.pose.position.z = points[i].z() + size; // Position text above point
                text_marker.pose.orientation.w = 1.0;
                text_marker.scale.z = 0.05; // Text height
                text_marker.color.r = 1.0f;
                text_marker.color.g = 1.0f;
                text_marker.color.b = 1.0f;
                text_marker.color.a = 1.0f;
                text_marker.text = labels[i];
                marker_array.markers.push_back(text_marker);
            }
        }

        // Publish all markers at once
        marker_pub_->publish(marker_array);

        RCLCPP_DEBUG(logger_, "Published %zu points with individual colors in namespace '%s'",
                     points.size(), ns.c_str());
    }

    void NBVVisualizer::publishSemanticPoints(
        const std::vector<GroundTruthSegment> &semantic_points,
        double size,
        float alpha,
        bool show_labels,
        const std::string &ns,
        const std::string &frame_id)
    {
        if (semantic_points.empty())
        {
            return;
        }

        // Extract positions and generate colors from class_id
        std::vector<octomap::point3d> positions;
        std::vector<std_msgs::msg::ColorRGBA> colors;
        std::vector<std::string> labels;

        positions.reserve(semantic_points.size());
        colors.reserve(semantic_points.size());
        if (show_labels)
        {
            labels.reserve(semantic_points.size());
        }

        for (const auto &sp : semantic_points)
        {
            positions.push_back(sp.position);
            colors.push_back(colorForLabel(sp.class_id, alpha));
            if (show_labels)
            {
                labels.push_back(sp.id);
            }
        }

        // Use the existing publishPoints function
        publishPoints(positions, colors, labels, size, alpha, ns, frame_id);

        RCLCPP_DEBUG(logger_, "Published %zu semantic points in namespace '%s'",
                     semantic_points.size(), ns.c_str());
    }

    void NBVVisualizer::publishMatchResults(
        const MatchResult &match_result,
        double size,
        float alpha,
        const std::string &ns,
        const std::string &frame_id)
    {
        // Keep track of all the gt points (correct, incorrect, both, and missed) and their labels/colors
        std::vector<GroundTruthSegment> correct_gt_points;
        std::vector<GroundTruthSegment> correct_incorrect_gt_points;
        std::vector<GroundTruthSegment> incorrect_gt_points;
        std::vector<GroundTruthSegment> missed_gt_points = match_result.unmatched_gt;

        // Keep track of all the gt points to publish and their corresponding colors and labels
        std::vector<octomap::point3d> gt_points;
        std::vector<std::string> labels;
        std::vector<std_msgs::msg::ColorRGBA> colors;

        // Define the colors for each category
        std_msgs::msg::ColorRGBA correct_color; // Green
        correct_color.r = 0.0f;
        correct_color.g = 1.0f;
        correct_color.b = 0.0f;
        correct_color.a = alpha;

        std_msgs::msg::ColorRGBA incorrect_color; // Red
        incorrect_color.r = 1.0f;
        incorrect_color.g = 0.0f;
        incorrect_color.b = 0.0f;
        incorrect_color.a = alpha;

        std_msgs::msg::ColorRGBA both_color; // Yellow (both correct and incorrect matches, indicating ambiguity)
        both_color.r = 1.0f;
        both_color.g = 1.0f;
        both_color.b = 0.0f;
        both_color.a = alpha;

        std_msgs::msg::ColorRGBA missed_color; // Blue (missed gt points)
        missed_color.r = 0.0f;
        missed_color.g = 0.0f;
        missed_color.b = 1.0f;
        missed_color.a = alpha;

        // Go through the correct gt points and add to the correct_gt_points vector only if they are not in the incorrect matches
        // Otherwise add to the correct_incorrect_gt_points vector (these are points that are both correctly and incorrectly matched, which indicates some ambiguity in the matching)
        for (const auto &correct_match : match_result.correct_matches)
        {
            // Check if this gt_point id is the same as any of the gt_point ids in the incorrect matches
            auto it = std::find_if(match_result.class_mismatches.begin(), match_result.class_mismatches.end(),
                                   [&correct_match](const Match &class_mismatch)
                                   {
                                       return correct_match.gt_point.id == class_mismatch.gt_point.id;
                                   });
            if (it != match_result.class_mismatches.end())
            {
                correct_incorrect_gt_points.push_back(correct_match.gt_point);
            }
            else
            {
                correct_gt_points.push_back(correct_match.gt_point);
            }
        }

        // Go through the incorrect matches and add to the incorrect_gt_points vector only if they are not in the correct matches
        // Ignore if they are in the correct matches since those points were already added to the correct_incorrect_gt_points vector
        for (const auto &class_mismatch : match_result.class_mismatches)
        {
            // Check if this gt_point id is the same as any of the gt_point ids in the correct matches
            auto it = std::find_if(match_result.correct_matches.begin(), match_result.correct_matches.end(),
                                   [&class_mismatch](const Match &correct_match)
                                   {
                                       return class_mismatch.gt_point.id == correct_match.gt_point.id;
                                   });
            if (it == match_result.correct_matches.end())
            {
                incorrect_gt_points.push_back(class_mismatch.gt_point);
            }
        }

        // Go through all of the correctly matched ground truth points and publish them in green
        for (const auto &gt_pt : correct_gt_points)
        {
            gt_points.push_back(gt_pt.position);
            labels.push_back(gt_pt.id);
            colors.push_back(correct_color);
        }

        // Go through all of the incorrectly matched ground truth points and publish them in red
        for (const auto &gt_pt : incorrect_gt_points)
        {
            gt_points.push_back(gt_pt.position);
            labels.push_back(gt_pt.id);
            colors.push_back(incorrect_color);
        }

        // Go through all of the correctly and incorrectly matched ground truth points and publish them in yellow (these are the points that are both correctly and incorrectly matched, which indicates some ambiguity in the matching)
        for (const auto &gt_pt : correct_incorrect_gt_points)
        {
            gt_points.push_back(gt_pt.position);
            labels.push_back(gt_pt.id);
            colors.push_back(both_color);
        }

        // Go through all of the missed ground truth points and publish them in blue
        for (const auto &gt_pt : missed_gt_points)
        {
            gt_points.push_back(gt_pt.position);
            labels.push_back(gt_pt.id);
            colors.push_back(missed_color);
        }

        // Publish all the gt points with their corresponding colors
        publishPoints(gt_points, colors, labels, size, alpha, ns, frame_id);
    }

    // bool NBVVisualizer::plotMetric(
    //     const std::vector<std::vector<double>> &y_data,
    //     const std::vector<double> &x_data,
    //     const std::vector<std::string> &series_labels,
    //     const std::string &plot_title,
    //     const std::string &x_title,
    //     const std::string &y_title,
    //     const std::vector<std::array<float, 3>> &colors,
    //     const std::string &save_path,
    //     double y_min,
    //     double y_max)
    // {
    //     if (y_data.empty())
    //     {
    //         return false;
    //     }

    //     // Validate input sizes
    //     if (!x_data.empty() && x_data.size() != y_data[0].size())
    //     {
    //         return false;
    //     }

    //     if (!series_labels.empty() && series_labels.size() != y_data.size())
    //     {
    //         return false;
    //     }

    //     // Create a mutable copy of colors and fill in missing colors
    //     std::vector<std::array<float, 3>> plot_colors = colors;
    //     if (plot_colors.size() < y_data.size())
    //     {
    //         for (size_t i = plot_colors.size(); i < y_data.size(); ++i)
    //         {
    //             // Add default color from label color map
    //             auto color_rgba = colorForLabel(static_cast<int>(i));
    //             plot_colors.push_back({color_rgba.r, color_rgba.g, color_rgba.b});
    //         }
    //     }

    //     try
    //     {
    //         using namespace matplot;

    //         size_t num_series = y_data.size();
    //         size_t num_points = y_data[0].size();

    //         // Create figure
    //         auto fig = figure(true);
    //         fig->size(1200, 800);

    //         // Use provided x_data or generate default positions
    //         std::vector<double> x_values;
    //         if (!x_data.empty())
    //         {
    //             x_values = x_data;
    //         }
    //         else
    //         {
    //             x_values.resize(num_points);
    //             for (size_t i = 0; i < num_points; ++i)
    //             {
    //                 x_values[i] = static_cast<double>(i);
    //             }
    //         }

    //         // Plot each series
    //         std::vector<matplot::line_handle> line_handles;
    //         hold(on);
    //         for (size_t i = 0; i < num_series; ++i)
    //         {
    //             auto h = plot(x_values, y_data[i]);
    //             h->line_width(2);

    //             if (!plot_colors.empty() && i < plot_colors.size())
    //             {
    //                 // color_array format is {alpha, r, g, b} where alpha=0 is fully opaque
    //                 h->color({0.0f, plot_colors[i][0], plot_colors[i][1], plot_colors[i][2]});
    //             }

    //             line_handles.push_back(h);
    //         }

    //         // Re-apply colors after all series are plotted, before save
    //         for (size_t i = 0; i < line_handles.size(); ++i)
    //         {
    //             if (!plot_colors.empty() && i < plot_colors.size())
    //             {
    //                 line_handles[i]->color({0.0f, plot_colors[i][0], plot_colors[i][1], plot_colors[i][2]});
    //             }
    //         }

    //         // Set labels and title
    //         title(plot_title);
    //         xlabel(x_title);
    //         ylabel(y_title);
    //         ylim({y_min, y_max});

    //         // Add legend if series labels provided
    //         if (!series_labels.empty())
    //         {
    //             legend(series_labels);
    //         }

    //         grid(on);

    //         // Save the figure
    //         try
    //         {
    //             fig->save(save_path);
    //         }
    //         catch (const std::exception &e)
    //         {
    //             return false;
    //         }

    //         // Show the plot (non-blocking)
    //         // show();

    //         return true;
    //     }
    //     catch (const std::exception &e)
    //     {
    //         return false;
    //     }
    // }

    // bool NBVVisualizer::plotClassMetrics(
    //     const std::vector<std::vector<ClassMetrics>> &metrics,
    //     const std::vector<double> &x_data,
    //     const std::string &plot_title,
    //     const std::string &x_data_label,
    //     const std::string &save_path)
    // {
    //     if (metrics.empty())
    //     {
    //         return false;
    //     }

    //     std::vector<std::vector<double>> y_data;
    //     std::vector<std::string> series_labels;

    //     int num_iterations = static_cast<int>(metrics.size());
    //     int num_classes = static_cast<int>(metrics[0].size());

    //     y_data.resize(num_classes * 3);
    //     for (auto &series : y_data)
    //         series.resize(num_iterations, 0.0);

    //     std::vector<std::array<float, 3>> plot_colors;
    //     plot_colors.reserve(num_classes * 3);

    //     // Convert RGB to HSV
    //     auto rgbToHsv = [](float r, float g, float b, float &h, float &s, float &v)
    //     {
    //         float cmax = std::max({r, g, b});
    //         float cmin = std::min({r, g, b});
    //         float diff = cmax - cmin;

    //         v = cmax;
    //         s = (cmax == 0.0f) ? 0.0f : diff / cmax;

    //         if (diff == 0.0f)       { h = 0.0f; }
    //         else if (cmax == r)     { h = 60.0f * std::fmod((g - b) / diff, 6.0f); }
    //         else if (cmax == g)     { h = 60.0f * ((b - r) / diff + 2.0f); }
    //         else                    { h = 60.0f * ((r - g) / diff + 4.0f); }

    //         if (h < 0.0f) h += 360.0f;
    //     };

    //     // Convert HSV to RGB
    //     auto hsvToRgb = [](float h, float s, float v, float &r, float &g, float &b)
    //     {
    //         float c = v * s;
    //         float x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    //         float m = v - c;

    //         if      (h < 60.0f)  { r = c; g = x; b = 0; }
    //         else if (h < 120.0f) { r = x; g = c; b = 0; }
    //         else if (h < 180.0f) { r = 0; g = c; b = x; }
    //         else if (h < 240.0f) { r = 0; g = x; b = c; }
    //         else if (h < 300.0f) { r = x; g = 0; b = c; }
    //         else                 { r = c; g = 0; b = x; }

    //         r += m; g += m; b += m;
    //     };

    //     // Rotate hue by offset degrees, keeping saturation and value the same
    //     auto rotateHue = [&](const std_msgs::msg::ColorRGBA &base_color, float hue_offset) -> std::array<float, 3>
    //     {
    //         float h, s, v;
    //         rgbToHsv(base_color.r, base_color.g, base_color.b, h, s, v);
    //         h = std::fmod(h + hue_offset + 360.0f, 360.0f);
    //         // Boost saturation so rotated colors don't look washed out
    //         s = std::min(1.0f, s * 1.1f);
    //         float r, g, b;
    //         hsvToRgb(h, s, v, r, g, b);
    //         return {r, g, b};
    //     };

    //     // Fill out the series labels and colors
    //     // Precision: base hue, Recall: +40 degrees, F1: -40 degrees
    //     for (size_t class_idx = 0; class_idx < static_cast<size_t>(num_classes); ++class_idx)
    //     {
    //         auto base_color = colorForLabel(static_cast<int>(class_idx), 1.0f);

    //         plot_colors.push_back(rotateHue(base_color,   0.0f));  // Precision - base hue
    //         plot_colors.push_back(rotateHue(base_color,  40.0f));  // Recall    - +40 degrees
    //         plot_colors.push_back(rotateHue(base_color, -40.0f));  // F1 Score  - -40 degrees

    //         series_labels.push_back("Class " + std::to_string(class_idx) + " Precision");
    //         series_labels.push_back("Class " + std::to_string(class_idx) + " Recall");
    //         series_labels.push_back("Class " + std::to_string(class_idx) + " F1 Score");
    //     }

    //     // Pull the precision, recall, and F1 scores across each iteration
    //     for (size_t i = 0; i < metrics.size(); ++i)
    //     {
    //         const auto &iter_metrics = metrics[i];
    //         for (size_t class_idx = 0; class_idx < static_cast<size_t>(num_classes); ++class_idx)
    //         {
    //             const auto &class_metrics = iter_metrics[class_idx];
    //             y_data[class_idx * 3][i]     = class_metrics.precision;
    //             y_data[class_idx * 3 + 1][i] = class_metrics.recall;
    //             y_data[class_idx * 3 + 2][i] = class_metrics.f1_score;
    //         }
    //     }

    //     return plotMetric(y_data, x_data, series_labels, plot_title, x_data_label, "Metric Value", plot_colors, save_path, 0.0, 1.0);
    // }

    // bool NBVVisualizer::plotAllMetrics(
    //     const std::vector<EvaluationMetrics> &all_metrics,
    //     const std::string &dir_path)
    // {
    //     if (all_metrics.empty())
    //     {
    //         return false;
    //     }

    //     // Extract class metrics from all evaluations
    //     std::vector<std::vector<ClassMetrics>> class_metrics_over_time;
    //     std::vector<double> x_data_time;
    //     std::vector<double> x_data_iter;

    //     for (size_t i = 0; i < all_metrics.size(); ++i)
    //     {
    //         class_metrics_over_time.push_back(all_metrics[i].class_metrics);
    //         x_data_iter.push_back(static_cast<double>(i));
    //         x_data_time.push_back(all_metrics[i].time);
    //     }

    //     // Plot the metric as a function of iterations
    //     bool success_iter = plotClassMetrics(class_metrics_over_time, x_data_iter, 
    //         "Class Metrics Over Viewpoints", "Viewpoint", dir_path + "/nbv_class_metrics_viewpoints.png");
    //     // Plot the metric as a function of time
    //     bool success_time = plotClassMetrics(class_metrics_over_time, x_data_time, 
    //         "Class Metrics Over Time", "Time (s)", dir_path + "/nbv_class_metrics_time.png");

    //     // Extract voxel count data
    //     std::vector<double> occupied_voxels_data;
    //     std::vector<double> free_voxels_data;
    //     std::vector<double> total_voxels_data;
    //     for (const auto &metrics : all_metrics)
    //     {
    //         occupied_voxels_data.push_back(static_cast<double>(metrics.occupied_voxels));
    //         free_voxels_data.push_back(static_cast<double>(metrics.free_voxels));
    //         total_voxels_data.push_back(static_cast<double>(metrics.occupied_voxels + metrics.free_voxels));
    //     }

    //     // Plot voxel counts over iterations
    //     std::vector<std::vector<double>> voxel_data_iter = {occupied_voxels_data, free_voxels_data, total_voxels_data};
    //     std::vector<std::string> voxel_labels = {"Occupied Voxels", "Free Voxels", "Total Voxels"};
    //     std::vector<std::array<float, 3>> voxel_colors = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        
    //     double max_voxels = total_voxels_data.empty() ? 1.0 : *std::max_element(total_voxels_data.begin(), total_voxels_data.end());
    //     bool success_voxels_iter = plotMetric(voxel_data_iter, x_data_iter, voxel_labels, 
    //                                          "Voxel Counts Over Viewpoints", "Viewpoint", "Voxel Count", 
    //                                          voxel_colors, dir_path + "/voxel_counts_viewpoints.png", 0.0, max_voxels * 1.1);

    //     // Plot voxel counts over time
    //     bool success_voxels_time = plotMetric(voxel_data_iter, x_data_time, voxel_labels, 
    //                                          "Voxel Counts Over Time", "Time (s)", "Voxel Count", 
    //                                          voxel_colors, dir_path + "/voxel_counts_time.png", 0.0, max_voxels * 1.1);

    //     // Extract coverage data
    //     std::vector<double> coverage_data;
    //     for (const auto &metrics : all_metrics)
    //     {
    //         coverage_data.push_back(metrics.bbox_coverage);
    //     }

    //     // Plot coverage percentage over iterations
    //     std::vector<std::vector<double>> coverage_y_data = {coverage_data};
    //     std::vector<std::string> coverage_labels = {"Coverage %"};
    //     std::vector<std::array<float, 3>> coverage_colors = {{0.0f, 0.5f, 1.0f}};
        
    //     bool success_coverage_iter = plotMetric(coverage_y_data, x_data_iter, coverage_labels, 
    //                                            "Octomap Coverage Over Viewpoints", "Viewpoint", "Coverage (%)",
    //                                            coverage_colors, dir_path + "/coverage_viewpoints.png", 0.0, 100.0);

    //     // Plot coverage percentage over time
    //     bool success_coverage_time = plotMetric(coverage_y_data, x_data_time, coverage_labels, 
    //                                            "Octomap Coverage Over Time", "Time (s)", "Coverage (%)",
    //                                            coverage_colors, dir_path + "/coverage_time.png", 0.0, 100.0);

    //     return success_iter && success_time && success_voxels_iter && success_voxels_time && 
    //            success_coverage_iter && success_coverage_time;
    // }

    bool NBVVisualizer::logAllMetricsToCSV(
        const std::vector<EvaluationMetrics> &all_metrics,
        const std::string &dir_path,
        const std::string &file_name)
    {
        if (all_metrics.empty())
        {
            return false;
        }

        // Open file for writing
        std::ofstream csv_file(dir_path + "/" + file_name);
        if (!csv_file.is_open())
        {
            return false;
        }

        try
        {
            // Write CSV header
            csv_file << "Viewpoint,Time,Occupied_Voxels,Free_Voxels,Total_Voxels,Coverage_Percent,Class_ID,TP_Clusters,FP_Clusters,TP_Points,FN_Points,Precision,Recall,F1_Score\n";

            // Write data for each run and class
            for (size_t vp_idx = 0; vp_idx < all_metrics.size(); ++vp_idx)
            {
                const auto &eval_metrics = all_metrics[vp_idx];
                
                // If there are class metrics, write one row per class
                if (!eval_metrics.class_metrics.empty())
                {
                    for (const auto &class_metrics : eval_metrics.class_metrics)
                    {
                        csv_file << vp_idx << ","
                                 << std::fixed << std::setprecision(3) << eval_metrics.time << ","
                                 << eval_metrics.occupied_voxels << ","
                                 << eval_metrics.free_voxels << ","
                                 << (eval_metrics.occupied_voxels + eval_metrics.free_voxels) << ","
                                 << std::fixed << std::setprecision(2) << eval_metrics.bbox_coverage << ","
                                 << class_metrics.class_id << ","
                                 << class_metrics.tp_clusters << ","
                                 << class_metrics.fp_clusters << ","
                                 << class_metrics.tp_points << ","
                                 << class_metrics.fn_points << ","
                                 << std::fixed << std::setprecision(4) << class_metrics.precision << ","
                                 << std::fixed << std::setprecision(4) << class_metrics.recall << ","
                                 << std::fixed << std::setprecision(4) << class_metrics.f1_score << "\n";
                    }
                }
                else
                {
                    // If no class metrics, still log voxel and coverage data
                    csv_file << vp_idx << ","
                             << std::fixed << std::setprecision(3) << eval_metrics.time << ","
                             << eval_metrics.occupied_voxels << ","
                             << eval_metrics.free_voxels << ","
                             << (eval_metrics.occupied_voxels + eval_metrics.free_voxels) << ","
                             << std::fixed << std::setprecision(2) << eval_metrics.bbox_coverage << ","
                             << ",,,,,,,,\n";  // Empty fields for class-specific metrics
                }
            }

            csv_file.close();
            return true;
        }
        catch (const std::exception &e)
        {
            csv_file.close();
            return false;
        }
    }

} // namespace erwinia_os_nbv_planner
