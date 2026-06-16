#pragma once

/**
 * @file semantic_occupancy_map_tree.hpp
 * @brief Semantic OctoMap tree + thread-safe wrapper around it (ROS2 Humble / octomap)
 */

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <mutex>
#include <string>
#include <utility>

namespace octomap
{

    class SemanticOcTree;

    /**
     * @brief Node with occupancy (log-odds) + semantic label/confidence
     */
    class SemanticOcTreeNode : public OcTreeNode
    {
    public:
        friend class SemanticOcTree;

        SemanticOcTreeNode()
            : OcTreeNode(), class_id_(0), confidence_(0.0f)
        {
        }

        SemanticOcTreeNode(const SemanticOcTreeNode &rhs)
            : OcTreeNode(rhs), class_id_(rhs.class_id_), confidence_(rhs.confidence_)
        {
        }

        bool operator==(const SemanticOcTreeNode &rhs) const
        {
            return (rhs.value == value &&
                    rhs.class_id_ == class_id_ &&
                    rhs.confidence_ == confidence_);
        }

        void copyData(const SemanticOcTreeNode &from)
        {
            OcTreeNode::copyData(from);
            class_id_ = from.class_id_;
            confidence_ = from.confidence_;
        }

        // ---- Semantic accessors ----
        inline int32_t getClassId() const { return class_id_; }
        inline float getConfidence() const { return confidence_; }

        inline void setSemantic(int32_t class_id, float confidence)
        {
            class_id_ = class_id;
            confidence_ = confidence;
        }

        inline bool isSemanticSet() const
        {
            return (class_id_ != 0) && (confidence_ > 0.0f);
        }

        /**
         * @brief Semantic fusion with averaging for same label and mismatch penalty for different labels
         * @param class_id New semantic class ID
         * @param confidence New confidence value [0.0, 1.0]
         * @param confidence_boost Boost applied when labels match (default 0.05)
         * @param mismatch_penalty Penalty multiplier when labels differ (default 0.1)
         */
        inline void fuseSemanticMax(int32_t class_id, float confidence, 
                                   float confidence_boost = 0.1f, 
                                   float mismatch_penalty = 0.1f)
        {
            if (class_id_ == 0 || confidence_ <= 0.0f)
            {
                // No prior semantic information - directly assign
                class_id_ = class_id;
                confidence_ = confidence;
            }
            else if (class_id == class_id_)
            {
                // Same label - average confidence scores + small boost
                // float updated_confidence = (confidence_ + confidence) / 2.0f + confidence_boost;
                float updated_confidence = (confidence_ + confidence) / 2.0f * (1.0f + confidence_boost);
                confidence_ = std::min(1.0f, std::max(0.0f, updated_confidence));
            }
            else
            {
                // Different labels - choose the one with higher confidence
                if (confidence > confidence_)
                {
                    class_id_ = class_id;
                    confidence_ = confidence;
                }
                // Apply mismatch penalty to the chosen label's confidence
                confidence_ = std::min(1.0f, std::max(0.0f, confidence_ * (1.0f - mismatch_penalty)));
            }
        }

        // inline void fuseSemanticMax(int32_t class_id, float confidence,
        //                     float /*confidence_boost*/ = 0.05f,
        //                     float /*mismatch_penalty*/ = 0.1f)
        // {
        //     if (class_id_ == 0 || confidence_ <= 0.0f)
        //     {
        //         class_id_ = class_id;
        //         confidence_ = confidence;
        //         return;
        //     }

        //     if (class_id == class_id_)
        //     {
        //         // P(correct after two independent agreements) = 1 - (1-p_old)(1-p_new)
        //         confidence_ = 1.0f - (1.0f - confidence_) * (1.0f - confidence);
        //     }
        //     else
        //     {
        //         float old_conf = confidence_;
        //         if (confidence > confidence_)
        //         {
        //             class_id_ = class_id;
        //             confidence_ = confidence * (1.0f - old_conf);
        //         }
        //         else
        //         {
        //             confidence_ = old_conf * (1.0f - confidence);
        //         }
        //     }
        // }

        // ---- Child plumbing (CRITICAL to avoid slicing) ----
        SemanticOcTreeNode *createChild(unsigned int i);
        SemanticOcTreeNode *getChild(unsigned int i);
        const SemanticOcTreeNode *getChild(unsigned int i) const;

        /**
         * @brief Expand into 8 children, copying occupancy + semantic payload.
         */
        void expandNode();

        // ---- Semantic aggregation (optional) ----
        void updateSemanticChildren();
        std::pair<int32_t, float> getBestChildSemantic() const;

        // ---- File I/O for extra payload ----
        std::istream &readData(std::istream &s);
        std::ostream &writeData(std::ostream &s) const;

    protected:
        int32_t class_id_;
        float confidence_;
    };

    /**
     * @brief Occupancy octree with semantic payload per node
     *
     * Note (ROS2 Humble): OccupancyOcTreeBase::updateInnerOccupancy() is NOT virtual,
     * so we do not override it. Instead we provide an explicit semantic version.
     */
    class SemanticOcTree : public OccupancyOcTreeBase<SemanticOcTreeNode>
    {
    public:
        explicit SemanticOcTree(double resolution);

        SemanticOcTree *create() const override { return new SemanticOcTree(this->resolution); }
        std::string getTreeType() const override { return "SemanticOcTree"; }

        // ========================================================================
        // OVERRIDE: Fixed bounding box behavior for free space
        // ========================================================================
        
        /**
         * @brief Override insertPointCloud to use fixed bbox free space filtering
         * 
         * @param scan Input point cloud
         * @param sensor_origin Sensor origin
         * @param maxrange Maximum sensor range (-1 for unlimited)
         * @param lazy_eval Whether to defer tree updates
         * @param discretize Whether to discretize endpoints to voxel centers
         */
        virtual void insertPointCloud(const Pointcloud& scan,
                                     const point3d& sensor_origin,
                                     double maxrange = -1.0,
                                     bool lazy_eval = false,
                                     bool discretize = false);

        /**
         * @brief Integrate semantic at a voxel key using max fusion.
         *
         * @param require_node_exist If true, do not create nodes (skip if absent).
         *        This is ideal when you ONLY want semantics for voxels that became occupied by insertPointCloud.
         * @param lazy_eval If true, defer tree propagation until updateInnerOccupancySemantic() is called.
         * @param confidence_boost Boost applied when labels match
         * @param mismatch_penalty Penalty multiplier when labels differ
         */
        SemanticOcTreeNode *integrateNodeSemantic(const OcTreeKey &key,
                                                  int32_t class_id,
                                                  float confidence,
                                                  bool require_node_exist = true,
                                                  bool lazy_eval = false,
                                                  float confidence_boost = 0.05f,
                                                  float mismatch_penalty = 0.1f);

        SemanticOcTreeNode *integrateNodeSemantic(const point3d &p,
                                                  int32_t class_id,
                                                  float confidence,
                                                  bool require_node_exist = true,
                                                  bool lazy_eval = false,
                                                  float confidence_boost = 0.05f,
                                                  float mismatch_penalty = 0.1f)
        {
            OcTreeKey key;
            if (!this->coordToKeyChecked(p, key))
                return nullptr;
            return integrateNodeSemantic(key, class_id, confidence, require_node_exist, 
                                       lazy_eval, confidence_boost, mismatch_penalty);
        }

        /**
         * @brief Update inner occupancy (base behavior) + also propagate semantics upward.
         *
         * Call this after batch updates if you used lazy evaluation OR if you want
         * semantic values on inner nodes.
         */
        void updateInnerOccupancySemantic();

    protected:
        void updateInnerOccupancySemanticRecurs(SemanticOcTreeNode *node, unsigned int depth);

        class StaticMemberInitializer
        {
        public:
            StaticMemberInitializer();
            void ensureLinking() {}
        };

        static StaticMemberInitializer semanticOcTreeMemberInit_;
    };

} // namespace octomap

namespace erwinia_os_occupancy_map
{

    /**
     * @brief Thread-safe wrapper around octomap::SemanticOcTree, mirroring your OccupancyMapTree
     */
    class SemanticOccupancyMapTree : public octomap::SemanticOcTree
    {
    public:
        explicit SemanticOccupancyMapTree(double resolution);
        explicit SemanticOccupancyMapTree(const std::string &filename);
        
        // Copy constructor needed for .ot reconstruction
        SemanticOccupancyMapTree(const SemanticOccupancyMapTree &rhs) = default;
        SemanticOccupancyMapTree(const octomap::SemanticOcTree &rhs);

        virtual ~SemanticOccupancyMapTree() = default;

        // ---- Thread-safe locking ----
        void lockRead();
        void unlockRead();
        void lockWrite();
        void unlockWrite();

        using ReadLock = std::shared_lock<std::shared_mutex>;
        using WriteLock = std::unique_lock<std::shared_mutex>;

        ReadLock reading();
        WriteLock writing();

        // ---- Update callbacks ----
        void triggerUpdateCallback();
        void setUpdateCallback(const std::function<void()> &callback);

        // ---- Statistics ----
        size_t getNumOccupiedNodes() const;
        size_t getNumFreeNodes() const;
        size_t memoryUsage() const;

    private:
        mutable std::shared_mutex tree_mutex_;
        std::function<void()> update_callback_;
    };

    using SemanticOccupancyMapTreePtr = std::shared_ptr<SemanticOccupancyMapTree>;
    using SemanticOccupancyMapTreeConstPtr = std::shared_ptr<const SemanticOccupancyMapTree>;

    /**
     * @brief Extended msgToMap that handles SemanticOcTree in addition to standard types
     * 
     * This function checks the tree type ID in the message and creates the appropriate
     * tree type. Falls back to standard octomap_msgs conversion for non-semantic trees.
     */
    octomap::AbstractOcTree* semanticMsgToMap(const octomap_msgs::msg::Octomap& msg);

} // namespace erwinia_os_occupancy_map
