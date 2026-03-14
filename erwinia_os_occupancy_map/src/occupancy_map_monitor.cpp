/**
 * @file occupancy_map_monitor.cpp
 * @brief Implementation of occupancy map coordinator
 */

#include "erwinia_os_occupancy_map/occupancy_map_monitor.hpp"
#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"

namespace erwinia_os_occupancy_map
{

    OccupancyMapMonitor::OccupancyMapMonitor(
        const rclcpp::Node::SharedPtr &node,
        const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
        const OccupancyMapParameters &params,
        bool use_semantic)
        : node_(node), tf_buffer_(tf_buffer), params_(params), logger_(node_->get_logger()), 
          active_(false), use_semantic_(use_semantic)
    {
        if (use_semantic_)
        {
            // Create semantic tree
            semantic_tree_ = std::make_shared<SemanticOccupancyMapTree>(params_.resolution);
            tree_ = nullptr;
            
            // Apply bounding box if enabled
            if (params_.use_bounding_box) {
                semantic_tree_->setBBXMin(params_.bbx_min);
                semantic_tree_->setBBXMax(params_.bbx_max);
                semantic_tree_->useBBXLimit(true);
                
                RCLCPP_INFO(logger_, "Bounding box enabled:");
                RCLCPP_INFO(logger_, "  Min: [%.2f, %.2f, %.2f]", 
                            params_.bbx_min.x(), params_.bbx_min.y(), params_.bbx_min.z());
                RCLCPP_INFO(logger_, "  Max: [%.2f, %.2f, %.2f]", 
                            params_.bbx_max.x(), params_.bbx_max.y(), params_.bbx_max.z());
            }
            
            RCLCPP_INFO(logger_, "OccupancyMapMonitor created (resolution: %.3f m, mode: SEMANTIC)", params_.resolution);
        }
        else
        {
            // Create standard tree
            tree_ = std::make_shared<OccupancyMapTree>(params_.resolution);
            tree_const_ = tree_;
            semantic_tree_ = nullptr;

            // Configure octree parameters
            tree_->setProbHit(params_.prob_hit);
            tree_->setProbMiss(params_.prob_miss);
            tree_->setClampingThresMin(params_.clamp_min);
            tree_->setClampingThresMax(params_.clamp_max);
            tree_->setOccupancyThres(params_.occupancy_threshold);

            // Apply bounding box if enabled
            if (params_.use_bounding_box) {
                tree_->setBBXMin(params_.bbx_min);
                tree_->setBBXMax(params_.bbx_max);
                tree_->useBBXLimit(true);
                
                RCLCPP_INFO(logger_, "Bounding box enabled:");
                RCLCPP_INFO(logger_, "  Min: [%.2f, %.2f, %.2f]", 
                            params_.bbx_min.x(), params_.bbx_min.y(), params_.bbx_min.z());
                RCLCPP_INFO(logger_, "  Max: [%.2f, %.2f, %.2f]", 
                            params_.bbx_max.x(), params_.bbx_max.y(), params_.bbx_max.z());
            }

            RCLCPP_INFO(logger_, "OccupancyMapMonitor created (resolution: %.3f m, mode: STANDARD)", params_.resolution);
        }
    }

    OccupancyMapMonitor::OccupancyMapMonitor(
        const rclcpp::Node::SharedPtr &node,
        const OccupancyMapParameters &params,
        bool use_semantic)
        : node_(node), tf_buffer_(nullptr), params_(params), logger_(node_->get_logger()), 
          active_(false), use_semantic_(use_semantic)
    {
        if (use_semantic_)
        {
            // Create semantic tree
            semantic_tree_ = std::make_shared<SemanticOccupancyMapTree>(params_.resolution);
            tree_ = nullptr;
            
            // Apply bounding box if enabled
            if (params_.use_bounding_box) {
                semantic_tree_->setBBXMin(params_.bbx_min);
                semantic_tree_->setBBXMax(params_.bbx_max);
                semantic_tree_->useBBXLimit(true);
                
                RCLCPP_INFO(logger_, "Bounding box enabled:");
                RCLCPP_INFO(logger_, "  Min: [%.2f, %.2f, %.2f]", 
                            params_.bbx_min.x(), params_.bbx_min.y(), params_.bbx_min.z());
                RCLCPP_INFO(logger_, "  Max: [%.2f, %.2f, %.2f]", 
                            params_.bbx_max.x(), params_.bbx_max.y(), params_.bbx_max.z());
            }
            
            RCLCPP_INFO(logger_, "OccupancyMapMonitor created (resolution: %.3f m, mode: SEMANTIC)", params_.resolution);
        }
        else
        {
            // Create standard tree
            tree_ = std::make_shared<OccupancyMapTree>(params_.resolution);
            tree_const_ = tree_;
            semantic_tree_ = nullptr;

            // Configure octree parameters
            tree_->setProbHit(params_.prob_hit);
            tree_->setProbMiss(params_.prob_miss);
            tree_->setClampingThresMin(params_.clamp_min);
            tree_->setClampingThresMax(params_.clamp_max);
            tree_->setOccupancyThres(params_.occupancy_threshold);

            // Apply bounding box if enabled
            if (params_.use_bounding_box) {
                tree_->setBBXMin(params_.bbx_min);
                tree_->setBBXMax(params_.bbx_max);
                tree_->useBBXLimit(true);
                
                RCLCPP_INFO(logger_, "Bounding box enabled:");
                RCLCPP_INFO(logger_, "  Min: [%.2f, %.2f, %.2f]", 
                            params_.bbx_min.x(), params_.bbx_min.y(), params_.bbx_min.z());
                RCLCPP_INFO(logger_, "  Max: [%.2f, %.2f, %.2f]", 
                            params_.bbx_max.x(), params_.bbx_max.y(), params_.bbx_max.z());
            }

            RCLCPP_INFO(logger_, "OccupancyMapMonitor created (resolution: %.3f m, mode: STANDARD)", params_.resolution);
        }
    }

    OccupancyMapMonitor::~OccupancyMapMonitor()
    {
        stopMonitor();
    }

    // ============================================================================
    // Control
    // ============================================================================

    void OccupancyMapMonitor::startMonitor()
    {
        if (active_)
        {
            RCLCPP_WARN(logger_, "Monitor already active");
            return;
        }

        // Start all updaters
        for (auto &updater : updaters_)
        {
            updater->start();
        }

        active_ = true;
        RCLCPP_INFO(logger_, "OccupancyMapMonitor started with %zu updater(s)", updaters_.size());
    }

    void OccupancyMapMonitor::stopMonitor()
    {
        if (!active_)
        {
            return;
        }

        // Stop all updaters
        for (auto &updater : updaters_)
        {
            updater->stop();
        }

        active_ = false;
        RCLCPP_INFO(logger_, "OccupancyMapMonitor stopped");
    }

    // ============================================================================
    // Updater management
    // ============================================================================

    void OccupancyMapMonitor::addUpdater(const OccupancyMapUpdaterPtr &updater)
    {
        if (!updater)
        {
            RCLCPP_ERROR(logger_, "Attempted to add null updater");
            return;
        }

        updater->setMonitor(this);
        updaters_.push_back(updater);

        RCLCPP_INFO(logger_, "Added updater: %s", updater->getType().c_str());
    }

    void OccupancyMapMonitor::clearUpdaters()
    {
        if (active_)
        {
            stopMonitor();
        }

        updaters_.clear();
        RCLCPP_INFO(logger_, "All updaters cleared");
    }

    // ============================================================================
    // Callbacks
    // ============================================================================

    void OccupancyMapMonitor::setUpdateCallback(const std::function<void()> &callback)
    {
        if (semantic_tree_)
        {
            semantic_tree_->setUpdateCallback(callback);
        }
        else if (tree_)
        {
            tree_->setUpdateCallback(callback);
        }
    }

    // ============================================================================
    // Services
    // ============================================================================

    bool OccupancyMapMonitor::saveMap(const std::string &filename)
    {
        RCLCPP_INFO(logger_, "Saving map to: %s", filename.c_str());

        bool success = false;

        try
        {
            if (semantic_tree_)
            {
                semantic_tree_->lockRead();
                success = semantic_tree_->writeBinary(filename);
                semantic_tree_->unlockRead();
            }
            else if (tree_)
            {
                tree_->lockRead();
                success = tree_->writeBinary(filename);
                tree_->unlockRead();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to save map: %s", e.what());
            success = false;
        }

        if (success)
        {
            RCLCPP_INFO(logger_, "Map saved successfully");
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to save map");
        }

        return success;
    }

    bool OccupancyMapMonitor::loadMap(const std::string &filename)
    {
        RCLCPP_INFO(logger_, "Loading map from: %s", filename.c_str());

        bool success = false;

        try
        {
            if (semantic_tree_)
            {
                // TODO: Implement semantic tree loading if needed
                RCLCPP_ERROR(logger_, "Loading semantic trees not yet implemented");
                return false;
            }
            else if (tree_)
            {
                tree_->lockWrite();
                // Read binary octree file
                auto *loaded_tree = dynamic_cast<octomap::OcTree *>(octomap::AbstractOcTree::read(filename));
                if (loaded_tree)
                {
                    // Create new OccupancyMapTree and swap it in
                    auto new_tree = std::make_shared<OccupancyMapTree>(params_.resolution);

                    // Copy nodes from loaded tree to new tree
                    for (auto it = loaded_tree->begin_leafs(); it != loaded_tree->end_leafs(); ++it)
                    {
                        new_tree->updateNode(it.getCoordinate(), it->getLogOdds());
                    }

                    // Replace tree
                    tree_ = new_tree;

                    delete loaded_tree;
                    success = true;
                }
                tree_->unlockWrite();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger_, "Failed to load map: %s", e.what());
            success = false;
        }

        if (success && tree_)
        {
            tree_->triggerUpdateCallback();
            RCLCPP_INFO(logger_, "Map loaded successfully");
        }
        else if (!success)
        {
            RCLCPP_ERROR(logger_, "Failed to load map");
        }

        return success;
    }

    void OccupancyMapMonitor::resetMap()
    {
        RCLCPP_INFO(logger_, "Resetting map");

        if (semantic_tree_)
        {
            semantic_tree_->lockWrite();
            semantic_tree_->clear();
            semantic_tree_->unlockWrite();
            semantic_tree_->triggerUpdateCallback();
        }
        else if (tree_)
        {
            tree_->lockWrite();
            tree_->clear();
            tree_->unlockWrite();
            tree_->triggerUpdateCallback();
        }
    }

} // namespace erwinia_os_occupancy_map
