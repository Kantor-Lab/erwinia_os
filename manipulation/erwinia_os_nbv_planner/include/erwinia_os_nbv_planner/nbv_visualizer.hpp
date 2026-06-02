/**
 * @file nbv_visualizer.hpp
 * @brief RViz visualization for NBV planning components
 */

#pragma once

#include "erwinia_os_nbv_planner/nbv_types.hpp"
#include "erwinia_os_nbv_planner/viewpoint.hpp"
#include "erwinia_os_nbv_planner/conversions.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <octomap/octomap.h>

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Geometry>
#include <unordered_map>
#include <numeric>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <set>
#include <algorithm>

namespace erwinia_os_nbv_planner
{

    /**
     * @brief Publishes RViz markers for NBV planning visualization
     *
     * Creates marker arrays showing:
     * - Frontier voxels (free voxels adjacent to unknown space)
     * - Cluster assignments and centers
     * - Candidate viewpoint poses
     * - Selected next-best-view
     * - Target region
     */
    class NBVVisualizer
    {
    public:
        /**
         * @brief Constructor
         * @param node ROS2 node
         * @param map_frame Frame ID for markers (default: "world")
         * @param topic Topic name for marker array publishing (default: "nbv_markers")
         */
        NBVVisualizer(
            const rclcpp::Node::SharedPtr &node,
            const std::string &map_frame = "world",
            const std::string &topic = "nbv_markers");

        /**
         * @brief Publish voxels as colored cubes
         * @param voxels Voxel positions
         * @param voxel_size Size of each voxel cube
         * @param color RGBA color for all voxels
         * @param alpha Alpha value for transparency (default: 0.85)
         * @param ns Namespace for the markers (default: "voxels")
         * @param frame_id Frame ID for input voxels (default: empty = map_frame)
         */
        void publishVoxels(
            const std::vector<octomap::point3d> &voxels,
            double voxel_size,
            const std_msgs::msg::ColorRGBA &color,
            float alpha = 0.85f,
            const std::string &ns = "voxels",
            const std::string &frame_id = "");

        /**
         * @brief Publish voxels with individual colors per voxel
         * @param voxels Voxel positions
         * @param voxel_size Size of each voxel cube
         * @param colors Vector of colors (must match size of voxels)
         * @param alpha Alpha value for transparency (default: 0.85)
         * @param ns Namespace for the markers (default: "voxels")
         * @param frame_id Frame ID for input voxels (default: empty = map_frame)
         */
        void publishVoxels(
            const std::vector<octomap::point3d> &voxels,
            double voxel_size,
            const std::vector<std_msgs::msg::ColorRGBA> &colors,
            float alpha = 0.85f,
            const std::string &ns = "voxels",
            const std::string &frame_id = "");

        /**
         * @brief Publish clustered voxels with per-cluster colors
         * @param clusters Vector of clusters, each containing voxel positions
         * @param voxel_size Size of each voxel cube
         * @param plot_centers Whether to plot cluster centers as small 0.1 diameter spheres
         * @param alpha Alpha value for transparency (default: 0.85)
         * @param ns Namespace for the markers (default: "clustered_voxels")
         * @param frame_id Frame ID for input clusters (default: empty = map_frame)
         */
        void publishClusteredVoxels(
            const std::vector<Cluster> &clusters,
            double voxel_size,
            bool plot_centers = false,
            float alpha = 0.85f,
            const std::string &ns = "clustered_voxels",
            const std::string &frame_id = "");

        /**
         * @brief Publish clustered voxels with a single color for all clusters
         * @param clusters Vector of clusters, each containing voxel positions
         * @param voxel_size Size of each voxel cube
         * @param color Color to use for all clusters
         * @param plot_centers Whether to plot cluster centers
         * @param alpha Alpha value for transparency (default: 0.85)
         * @param ns Namespace for the markers
         * @param frame_id Frame ID for input clusters (default: empty = map_frame)
         */
        void publishClusteredVoxels(
            const std::vector<Cluster> &clusters,
            double voxel_size,
            const std_msgs::msg::ColorRGBA &color,
            bool plot_centers = false,
            float alpha = 0.85f,
            const std::string &ns = "clustered_voxels",
            const std::string &frame_id = "");

        /**
         * @brief Publish clustered voxels with individual colors per cluster
         * @param clusters Vector of clusters, each containing voxel positions
         * @param voxel_size Size of each voxel cube
         * @param colors Vector of colors (must match size of clusters)
         * @param plot_centers Whether to plot cluster centers
         * @param alpha Alpha value for transparency (default: 0.85)
         * @param ns Namespace for the markers
         * @param frame_id Frame ID for input clusters (default: empty = map_frame)
         */
        void publishClusteredVoxels(
            const std::vector<Cluster> &clusters,
            double voxel_size,
            const std::vector<std_msgs::msg::ColorRGBA> &colors,
            bool plot_centers = false,
            float alpha = 0.85f,
            const std::string &ns = "clustered_voxels",
            const std::string &frame_id = "");

        /**
         * @brief Publish geometry poses as axes
         * @param poses Candidate camera poses
         * @param axis_length Length of coordinate axes
         * @param axis_radius Thickness of axes
         * @param alpha Alpha value for transparency (default: 0.5)
         * @param ns Namespace for the markers (default: "coordinate")
         * @param frame_id Frame ID for input poses (default: empty = map_frame)
         */
        void publishCoordinates(
            const std::vector<geometry_msgs::msg::Pose> &poses,
            double axis_length = 0.15,
            double axis_radius = 0.01,
            float alpha = 0.5f,
            const std::string &ns = "coordinate",
            const std::string &frame_id = "");

        /**
         * @brief Publish geometry pose as axes
         * @param pose Selected NBV camera pose
         * @param axis_length Length of coordinate axes
         * @param axis_radius Thickness of axes
         * @param alpha Alpha value for transparency (default: 1.0)
         * @param ns Namespace for the marker (default: "coordinate")
         * @param frame_id Frame ID for input pose (default: empty = map_frame)
         */
        void publishCoordinate(
            const geometry_msgs::msg::Pose &pose,
            double axis_length = 0.15,
            double axis_radius = 0.01,
            float alpha = 1.0f,
            const std::string &ns = "coordinate",
            const std::string &frame_id = "");
        
        void publishCoordinate(
            const Eigen::Vector3d &position,
            const std::array<double, 4> &orientation,
            double axis_length = 0.15,
            double axis_radius = 0.01,
            float alpha = 1.0f,
            const std::string &ns = "coordinate",
            const std::string &frame_id = "");

        /**
         * @brief Publish viewpoint coordinate frame
         * @param viewpoint Viewpoint with position and orientation
         * @param axis_length Length of coordinate axes
         * @param axis_radius Thickness of axes
         * @param alpha Alpha value for transparency (default: 1.0)
         * @param ns Namespace for the marker (default: "viewpoint")
         * @param frame_id Frame ID for input viewpoint (default: empty = map_frame)
         */
        void publishViewpoint(
            const Viewpoint &viewpoint,
            double axis_length = 0.15,
            double axis_radius = 0.01,
            float alpha = 1.0f,
            const std::string &ns = "viewpoint",
            const std::string &frame_id = "");

        /**
         * @brief Publish target region as wireframe sphere
         * @param center Center of target region
         * @param radius Radius of target region
         * @param line_width Width of wireframe lines
         * @param ns Namespace for the marker (default: "target_region")
         * @param frame_id Frame ID for input center (default: empty = map_frame)
         */
        void publishTargetRegion(
            const octomap::point3d &center,
            double radius,
            double line_width = 0.02,
            const std::string &ns = "target_region",
            const std::string &frame_id = "");

        /**
         * @brief Publish bounding box as LINE_LIST marker
         * @param bbx_min Minimum corner of bounding box
         * @param bbx_max Maximum corner of bounding box
         * @param ns Namespace for the marker (default: "bounding_box")
         * @param line_width Width of the lines (default: 0.02)
         * @param color RGBA color for the box lines
         * @param frame_id Frame ID for input bounds (default: empty = map_frame)
         */
        void publishBoundingBox(
            const octomap::point3d &bbx_min,
            const octomap::point3d &bbx_max,
            const std::string &ns = "bounding_box",
            double line_width = 0.02,
            const std_msgs::msg::ColorRGBA &color = std_msgs::msg::ColorRGBA(),
            const std::string &frame_id = "");

        /**
         * @brief Publish a plane defined by corner points
         * @param corners Vector of corner points (typically 4 corners defining a quadrilateral)
         * @param ns Namespace for the marker (default: "plane")
         * @param line_width Width of the lines (default: 0.02)
         * @param color RGBA color for the plane lines
         * @param draw_grid Whether to draw a grid on the plane (default: false)
         * @param grid_divisions Number of grid divisions per axis (default: 10)
         * @param frame_id Frame ID for input corners (default: empty = map_frame)
         */
        void publishPlane(
            const std::vector<Eigen::Vector3d> &corners,
            const std::string &ns = "plane",
            double line_width = 0.02,
            const std_msgs::msg::ColorRGBA &color = std_msgs::msg::ColorRGBA(),
            const std::string &frame_id = "");

        /**
         * @brief Clear all visualizations in a namespace
         * @param ns_suffix Namespace suffix (empty = all namespaces)
         */
        void clearAll(const std::string &ns_suffix = "");

        /**
         * @brief Clear all marker namespaces at once
         */
        void clearAllMarkers();

        /**
         * @brief Publish a single point marker with optional text label
         * @param point 3D point position
         * @param id Unique ID for the marker
         * @param size Size of the point marker (default: 0.02m)
         * @param color Optional color for the point (default: green)
         * @param alpha Alpha value for transparency (default: 0.8)
         * @param ns Namespace for the marker (default: "points")
         * @param text_label Optional text label to display above the point
         * @param frame_id Frame ID for input point (default: empty = map_frame)
         */
        void publishPoint(
            const octomap::point3d &point,
            int id,
            double size = 0.02,
            const std_msgs::msg::ColorRGBA &color = std_msgs::msg::ColorRGBA(),
            float alpha = 0.8f,
            const std::string &ns = "points",
            const std::string &text_label = "",
            const std::string &frame_id = "");

        /**
         * @brief Publish multiple points as a single marker (more efficient)
         * @param points Vector of 3D point positions
         * @param size Size of each point (default: 0.02m)
         * @param color Optional color for all points (default: green)
         * @param alpha Alpha value for transparency (default: 0.8)
         * @param ns Namespace for the marker (default: "points")
         * @param frame_id Frame ID for input points (default: empty = map_frame)
         */
        void publishPoints(
            const std::vector<octomap::point3d> &points,
            double size = 0.02,
            const std_msgs::msg::ColorRGBA &color = std_msgs::msg::ColorRGBA(),
            float alpha = 0.8f,
            const std::string &ns = "points",
            const std::string &frame_id = "");

        /**
         * @brief Publish multiple points with individual colors and optional text labels
         * @param points Vector of 3D point positions
         * @param colors Vector of colors (must match size of points)
         * @param labels Optional vector of text labels (if non-empty, must match size of points)
         * @param size Size of each point sphere (default: 0.02m)
         * @param alpha Alpha value for transparency (default: 0.8)
         * @param ns Namespace for the markers (default: "points")
         * @param frame_id Frame ID for input points (default: empty = map_frame)
         */
        void publishPoints(
            const std::vector<octomap::point3d> &points,
            const std::vector<std_msgs::msg::ColorRGBA> &colors,
            const std::vector<std::string> &labels = {},
            double size = 0.02,
            float alpha = 0.8f,
            const std::string &ns = "points",
            const std::string &frame_id = "");

        /**
         * @brief Publish semantic points with colors based on class_id
         * @param semantic_points Vector of GT segments with id, class_id, and position
         * @param size Size of each point sphere (default: 0.02m)
         * @param alpha Alpha value for transparency (default: 0.8)
         * @param show_labels Whether to display ID labels above points (default: true)
         * @param ns Namespace for the markers (default: "semantic_points")
         * @param frame_id Frame ID for input points (default: empty = map_frame)
         */
        void publishSemanticPoints(
            const std::vector<GroundTruthSegment> &semantic_points,
            double size = 0.02,
            float alpha = 0.8f,
            bool show_labels = true,
            const std::string &ns = "semantic_points",
            const std::string &frame_id = "");

        /**
         * @brief Publish match results with different colors for correct matches, class mismatches, and incorrect matches
         * @param match_result MatchResult containing vectors of correct matches, class mismatches, and incorrect matches
         * @param point_size Size of each point sphere (default: 0.02m
         * @param alpha Alpha value for transparency (default: 0.8)
         * @param ns Namespace for the markers (default: "match_results")
         * @param frame_id Frame ID for input points (default: empty = map_frame)
         */
        void publishMatchResults(
            const MatchResult &match_result,
            double point_size = 0.02,
            float alpha = 0.8f,
            const std::string &ns = "match_results",
            const std::string &frame_id = "");

        // /**
        //  * @brief Plot generic data as line chart with customizable labels and colors
        //  * @param y_data 2D vector where each inner vector represents one series of y values
        //  * @param x_data X-axis values (shared across all series, optional - will use 0,1,2... if empty)
        //  * @param series_labels Labels for each series (legend entries)
        //  * @param plot_title Title of the plot
        //  * @param x_title X-axis label
        //  * @param y_title Y-axis label
        //  * @param colors Colors for each series (optional, uses defaults if empty)
        //  * @param save_path Path to save the plot (default: "/tmp/plot.png")
        //  * @param y_min Minimum y-axis value (default: 0.0)
        //  * @param y_max Maximum y-axis value (default: 1.0)
        //  * @return true if successful, false otherwise
        //  */
        // static bool plotMetric(
        //     const std::vector<std::vector<double>> &y_data,
        //     const std::vector<double> &x_data,
        //     const std::vector<std::string> &series_labels,
        //     const std::string &plot_title = "Metric",
        //     const std::string &x_title = "X",
        //     const std::string &y_title = "Y",
        //     const std::vector<std::array<float, 3>> &colors = {},
        //     const std::string &save_path = "/tmp/plot.png",
        //     double y_min = 0.0,
        //     double y_max = 1.0
        // );

        // /**
        //  * @brief Plot evaluation metrics from ClassMetrics structure
        //  * @param class_metrics 2D vector of ClassMetrics (outer: different runs, inner: classes)
        //  * @param x_data X-axis values
        //  * @param x_data_label X-axis label
        //  * @param plot_title Title of the plot (default: "Class Metrics")
        //  * @param save_path Path to save the plot (default: "/tmp/metrics.png")
        //  * @return true if successful, false otherwise
        //  */
        // static bool plotClassMetrics(
        //     const std::vector<std::vector<ClassMetrics>> &metrics,
        //     const std::vector<double> &x_data,
        //     const std::string &x_data_label,
        //     const std::string &plot_title = "Class Metrics",
        //     const std::string &save_path = "/tmp/metrics.png"
        // );

        // /**
        //  * @brief Plots evaluation metrics from EvaluationMetrics structure
        //  * @param all_metrics 2D vector of ClassMetrics (outer: different runs, inner: classes)
        //  * @param dir_path Path to save the plots (default: "/tmp/metrics.png")
        //  * @return true if successful, false otherwise
        //  */
        // static bool plotAllMetrics(
        //     const std::vector<EvaluationMetrics> &all_metrics,
        //     const std::string &dir_path = "/tmp"
        // );

        /**
         * @brief Log metrics to csv file
         * @param all_metrics 2D vector of EvaluationMetrics (outer: different runs, inner: classes)
         * @param file_path Path to the csv file
         * @return true if successful, false otherwise
         */
        static bool logAllMetricsToCSV(
            const std::vector<EvaluationMetrics> &all_metrics,
            const std::string &dir_path = "/tmp",
            const std::string &file_name = "nbv_metrics.csv"
        );

        /**
         * @brief Clear specific visualization namespace
         */
        void clearVoxels();
        void clearClusteredVoxels();
        void clearClusterCenters();
        void clearCandidateViews();
        void clearBestView();
        void clearTargetRegion();

        /**
         * @brief Generate deterministic color from cluster label
         * @param label Class or cluster label
         * @param alpha Alpha value for transparency (default: 0.85)
         * @return RGBA color corresponding to the label
         */
        static std_msgs::msg::ColorRGBA colorForLabel(int label, float alpha = 0.85f);

        /**
         * @brief Get the current map frame
         * @return Current map frame ID
         */
        std::string getMapFrame() const { return map_frame_; }

        /**
         * @brief Set the map frame for marker publishing
         * @param frame_id New map frame ID
         */
        void setMapFrame(const std::string &frame_id) { map_frame_ = frame_id; }

    private:

        /**
         * @brief Create wireframe sphere marker
         */
        visualization_msgs::msg::Marker createWireframeSphere(
            const octomap::point3d &center,
            double radius,
            double line_width,
            const std_msgs::msg::ColorRGBA &color,
            const std::string &ns);

        /**
         * @brief Create coordinate axis triad marker (LINE_LIST for single marker compatibility)
         */
        visualization_msgs::msg::Marker createAxisTriad(
            const geometry_msgs::msg::Pose &pose,
            int id,
            double axis_length,
            double axis_radius,
            float alpha = 1.0f,
            const std::string &ns = "coordinate");

        /**
         * @brief Create coordinate axis triad using cylinder markers for each axis
         * @return Vector of 3 cylinder markers (X, Y, Z axes)
         */
        std::vector<visualization_msgs::msg::Marker> createAxisTriadCylinders(
            const geometry_msgs::msg::Pose &pose,
            int base_id,
            double axis_length,
            double axis_radius,
            float alpha = 1.0f,
            const std::string &ns = "coordinate");



        rclcpp::Node::SharedPtr node_;
        std::string map_frame_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Logger logger_;
    };

    using NBVVisualizerPtr = std::shared_ptr<NBVVisualizer>;

} // namespace erwinia_os_nbv_planner
