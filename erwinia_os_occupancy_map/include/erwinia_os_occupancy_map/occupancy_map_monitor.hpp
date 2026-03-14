/**
 * @file occupancy_map_monitor.hpp
 * @brief Manages occupancy map updates from sensors
 *
 * Coordinates sensor data collection and octree updates.
 * Similar to MoveIt's OccupancyMapMonitor but simplified.
 */

#pragma once

#include "erwinia_os_occupancy_map/occupancy_map_tree.hpp"
#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"
#include "erwinia_os_occupancy_map/occupancy_map_updater.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>
#include <vector>

namespace erwinia_os_occupancy_map
{

    /**
     * @brief Parameters for occupancy map configuration
     */
    struct OccupancyMapParameters
    {
        double resolution;                   ///< Voxel size (m)
        std::string map_frame;               ///< Fixed frame for map
        double max_range;                    ///< Maximum sensor range (m)
        double min_range;                    ///< Minimum sensor range (m)
        double prob_hit;                     ///< Occupancy probability on hit
        double prob_miss;                    ///< Occupancy probability on miss
        double clamp_min;                    ///< Minimum clamping threshold
        double clamp_max;                    ///< Maximum clamping threshold
        double occupancy_threshold;          ///< Threshold for considering voxel occupied
        // bool filter_ground_plane;            ///< Remove ground plane
        // double ground_distance_threshold;    ///< Distance to ground (m)

        // Bounding box parameters (optional)
        bool use_bounding_box;
        octomap::point3d bbx_min;
        octomap::point3d bbx_max;

        // Semantic fusion parameters
        float semantic_confidence_boost;   ///< Boost when labels match (default 0.05)
        float semantic_mismatch_penalty;   ///< Penalty when labels differ (default 0.1)
    };

    /**
     * @brief Monitors and updates occupancy map from sensor streams
     *
     * Design:
     * - Owns the octree
     * - Manages sensor updaters (pointcloud, depth, etc.)
     * - Thread-safe access to map
     * - Provides services (save/load)
     * - Can be used standalone or with MoveIt
     */
    class OccupancyMapMonitor
    {
    public:
        /**
         * @brief Construct monitor with ROS node
         * @param node ROS2 node handle
         * @param tf_buffer TF2 buffer for transforms
         * @param params Configuration parameters
         * @param use_semantic If true, creates semantic tree; otherwise standard tree
         */
        OccupancyMapMonitor(
            const rclcpp::Node::SharedPtr &node,
            const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
            const OccupancyMapParameters &params,
            bool use_semantic = false);

        /**
         * @brief Simplified constructor without TF buffer
         * @param node ROS2 node handle
         * @param params Configuration parameters
         * @param use_semantic If true, creates semantic tree; otherwise standard tree
         */
        OccupancyMapMonitor(
            const rclcpp::Node::SharedPtr &node,
            const OccupancyMapParameters &params,
            bool use_semantic = false);

        /**
         * @brief Destructor - stops monitoring
         */
        virtual ~OccupancyMapMonitor();

        // ========================================================================
        // Control
        // ========================================================================

        /**
         * @brief Start monitoring (begin processing sensor data)
         */
        void startMonitor();

        /**
         * @brief Stop monitoring (pause processing)
         */
        void stopMonitor();

        /**
         * @brief Check if currently active
         */
        bool isActive() const { return active_; }

        // ========================================================================
        // Map access
        // ========================================================================

        /**
         * @brief Get mutable octree pointer
         */
        const OccupancyMapTreePtr &getOcTreePtr() { return tree_; }

        /**
         * @brief Get const octree pointer
         */
        const OccupancyMapTreeConstPtr &getOcTreeConstPtr() const { return tree_const_; }

        /**
         * @brief Get map tree (alias for getOcTreePtr for convenience)
         */
        const OccupancyMapTreePtr &getMapTree() { return tree_; }

        /**
         * @brief Get semantic tree (returns nullptr if not semantic mode)
         */
        const SemanticOccupancyMapTreePtr &getSemanticMapTree() { return semantic_tree_; }

        /**
         * @brief Get map frame name
         */
        const std::string &getMapFrame() const { return params_.map_frame; }

        /**
         * @brief Get parameters
         */
        const OccupancyMapParameters &getParameters() const { return params_; }

        // ========================================================================
        // Updater management
        // ========================================================================

        /**
         * @brief Add a sensor updater
         * @param updater Sensor-specific updater instance
         */
        void addUpdater(const OccupancyMapUpdaterPtr &updater);

        /**
         * @brief Remove all updaters
         */
        void clearUpdaters();

        /**
         * @brief Get number of updaters
         */
        size_t getNumUpdaters() const { return updaters_.size(); }

        // ========================================================================
        // Callbacks
        // ========================================================================

        /**
         * @brief Set callback triggered when map updates
         * @param callback Function to call on update
         */
        void setUpdateCallback(const std::function<void()> &callback);

        // ========================================================================
        // Services
        // ========================================================================

        /**
         * @brief Save map to file
         * @param filename Path to save (.bt or .ot)
         * @return true if successful
         */
        bool saveMap(const std::string &filename);

        /**
         * @brief Load map from file
         * @param filename Path to load
         * @return true if successful
         */
        bool loadMap(const std::string &filename);

        /**
         * @brief Reset map (clear all voxels)
         */
        void resetMap();

    private:
        // ROS
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        
        // Map (params_ must come before logger_ to match initialization order)
        OccupancyMapParameters params_;
        rclcpp::Logger logger_;
        OccupancyMapTreePtr tree_;                        ///< Standard tree (nullptr in semantic mode)
        OccupancyMapTreeConstPtr tree_const_;
        SemanticOccupancyMapTreePtr semantic_tree_;      ///< Semantic tree (nullptr in standard mode)

        // Updaters
        std::vector<OccupancyMapUpdaterPtr> updaters_;

        // State
        bool active_;
        bool use_semantic_;  ///< True if using semantic tree
    };

} // namespace erwinia_os_occupancy_map
