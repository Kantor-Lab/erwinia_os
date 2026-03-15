#pragma once

#include "erwinia_os_occupancy_map/msg/custom_octomap.hpp"
#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"
#include "erwinia_os_nbv_planner/nbv_types.hpp"
#include "erwinia_os_nbv_planner/conversions.hpp"

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <shared_mutex>
#include <memory>
#include <vector>
#include <variant>
#include <map>

namespace erwinia_os_nbv_planner
{
    enum class OctoMapType
    {
        STANDARD,
        SEMANTIC
    };

    class OctoMapInterface
    {
    public:
        OctoMapInterface(const rclcpp::Node::SharedPtr &node,
                         const std::string &octomap_topic = "/nbv/octomap_binary",
                         bool transient_local = true);

        // Common operations (work with both tree types)
        std::shared_ptr<octomap::AbstractOcTree> getTreeSnapshot() const;

        std::vector<octomap::point3d> findFrontiers(int min_unknown_neighbors = 1,
                                                    bool use_bbox = false) const;

        // Returns centers of occupied semantic voxels with low confidence.
        // These can be treated as "semantic frontiers".
        std::vector<octomap::point3d> getUncertainVoxels(
            float max_confidence = 0.5f,
            bool use_bbox = true,
            bool count_background = true) const;

        std::vector<Cluster> kmeansCluster(const std::vector<octomap::point3d> &points,
                                           int n_clusters = 0,
                                           int max_iters = 50,
                                           double tol = 1e-4) const;

        bool isTreeAvailable() const {
            std::shared_lock lk(mtx_);
            return std::visit([](auto&& tree) { return tree != nullptr; }, tree_);
        }
        bool hasBoundingBox() const {
            std::shared_lock lk(mtx_);
            return has_valid_bbox_;
        }
        double getResolution() const {
            std::shared_lock lk(mtx_);
            return resolution_;
        }
        bool getBoundingBox(octomap::point3d &min_out, octomap::point3d &max_out) const {
            if (!has_valid_bbox_) {
                return false;
            }
            min_out = bbox_min_;
            max_out = bbox_max_;
            return true;
        }
        bool isVoxelOccupied(const octomap::point3d &point) const;
        bool searchOctree(const octomap::point3d &point, octomap::OcTreeNode *&node_out) const;
        int getOctreeDepth() const;
        rclcpp::Time getLastUpdateTime() const {
            std::shared_lock lk(mtx_);
            return last_update_time_;
        }

        // Tree type info
        OctoMapType getTreeType() const {
            std::shared_lock lk(mtx_);
            return tree_type_;
        }
        bool isSemanticTree() const {
            return getTreeType() == OctoMapType::SEMANTIC;
        }
        std::string getTreeTypeString() const {
            std::shared_lock lk(mtx_);
            return tree_type_ == OctoMapType::SEMANTIC ? "SemanticOcTree" : "OcTree";
        }

        // Semantic-specific operations (return false if not semantic tree)
        bool getVoxelSemantic(const octomap::point3d &point,
                              int32_t &class_id,
                              float &confidence) const;

        std::vector<octomap::point3d> findFrontiersByClass(int32_t class_id,
                                                           float min_confidence = 0.5f,
                                                           int min_unknown_neighbors = 1,
                                                           bool use_bbox = false) const;

        // Get counts of occupied voxels per semantic class
        // Returns map of class_id -> count. Returns empty map if not semantic tree.
        std::map<int32_t, size_t> getSemanticClassCounts() const;

        // Type-safe accessors
        std::shared_ptr<octomap::OcTree> getStandardTree() const {
            std::shared_lock lk(mtx_);
            if (tree_type_ == OctoMapType::STANDARD) {
                return std::get<std::shared_ptr<octomap::OcTree>>(tree_);
            }
            return nullptr;
        }
        std::shared_ptr<octomap::SemanticOcTree> getSemanticTree() const {
            std::shared_lock lk(mtx_);
            if (tree_type_ == OctoMapType::SEMANTIC) {
                return std::get<std::shared_ptr<octomap::SemanticOcTree>>(tree_);
            }
            return nullptr;
        }

        // Ground truth evaluation
        /**
         * @brief Load ground truth semantic points from YAML file
         * @param yaml_file_path Path to the YAML file containing marker positions
         * @return true if loaded successfully, false otherwise
         */
        bool loadGroundTruthSemantics(const std::string &yaml_file_path);

        /**
         * @brief Match semantic clusters to ground truth points
         * @param clusters Vector of semantic clusters to match
         * @param threshold_radius Maximum distance between cluster centroid and ground truth point for a match
         * @param verbose If true, print detailed matching information
         * @return MatchResult containing correct matches, class mismatches, and unmatched points/clusters
         */
        MatchResult matchClustersToGroundTruth(const std::vector<Cluster> &clusters,
                                               double threshold_radius = 0.2,
                                               bool verbose = false) const;

        /**
         * @brief Compute and print evaluation metrics from match results
         * @param result MatchResult from matchClustersToGroundTruth
         * @param verbose If true, print detailed lists of unmatched points and clusters
         */
        std::vector<ClassMetrics> evaluateMatchResults(const MatchResult &result, bool verbose = false) const;

        /**
         * @brief Evaluate semantic octomap against ground truth
         * @param threshold_radius Maximum distance between cluster centroid and ground truth point for a match
         * @param verbose If true, print detailed evaluation information
         */
        std::vector<ClassMetrics> evaluateSemanticOctomap(double threshold_radius = 0.2, bool verbose = false);

        /**
         * @brief Get the loaded ground truth points
         * @return Vector of ground truth semantic points
         */
        const std::vector<SemanticPoint> &getGroundTruthPoints() const { return gt_points_; }

        /**
         * @brief Get the frame ID of the loaded ground truth points
         * @return Frame ID string (empty if no GT loaded)
         */
        std::string getGroundTruthFrameId() const { return gt_frame_id_; }

        /**
         * @brief Get the frame ID of the current octomap
         * @return Frame ID string (empty if no octomap loaded)
         */
        std::string getOctomapFrameId() const { 
            std::shared_lock lk(mtx_);
            return octomap_frame_id_;
        }

        /**
         * @brief Cluster semantic voxels by class using connected component analysis
         * @param verbose If true, print cluster statistics
         * @return Vector of semantic clusters (excludes background class)
         */
        std::vector<Cluster> clusterSemanticVoxels(bool verbose = false) const;

        /**
         * @brief Get the number of occupied and free voxels in the octomap
         * @return std::pair<size_t, size_t> where first is occupied count and second is free count
         */
        std::pair<size_t, size_t> getVoxelCounts() const;

        /**
         * @brief Calculate coverage percentage of the bounding box that has been mapped
         * @return Coverage percentage (0-100) based on mapped voxels / theoretical max voxels in bbox, or 0.0 if no bbox or no voxels
         */
        double calculateCoverage() const;

        /**
         * @brief Print statistics about the current octomap
         * Logs resolution, node counts, occupied/free voxels, and semantic class info (if semantic tree)
         */
        void printOctomapStats() const;

    private:
        void onOctomap(const erwinia_os_occupancy_map::msg::CustomOctomap::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<erwinia_os_occupancy_map::msg::CustomOctomap>::SharedPtr sub_;

        mutable std::shared_mutex mtx_;

        // Store as variant to support both tree types without type erasure overhead
        std::variant<
            std::shared_ptr<octomap::OcTree>,
            std::shared_ptr<octomap::SemanticOcTree>>
            tree_;

        OctoMapType tree_type_{OctoMapType::STANDARD};
        double resolution_{0.0};
        octomap::point3d bbox_min_{0.0, 0.0, 0.0};
        octomap::point3d bbox_max_{0.0, 0.0, 0.0};
        bool has_valid_bbox_{false};
        rclcpp::Time last_update_time_;

        // Ground truth data
        std::vector<SemanticPoint> gt_points_;
        std::vector<int> gt_classes_;
        std::string gt_frame_id_;  // Frame ID for ground truth points
        
        // Octomap frame
        std::string octomap_frame_id_;  // Frame ID for octomap
        
        // TF2 for coordinate transformations
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };

} // namespace erwinia_os_nbv_planner