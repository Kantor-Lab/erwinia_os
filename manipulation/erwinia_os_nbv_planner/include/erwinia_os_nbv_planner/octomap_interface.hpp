#pragma once

#include "erwinia_os_occupancy_map/msg/custom_octomap.hpp"
#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"
#include "erwinia_os_nbv_planner/nbv_types.hpp"
#include "erwinia_os_nbv_planner/conversions.hpp"

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
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

        std::vector<VoxelSample> getVoxelSamples(bool include_free = false) const;

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
        
        // Octomap frame
        std::string octomap_frame_id_;  // Frame ID for octomap
    };

} // namespace erwinia_os_nbv_planner
