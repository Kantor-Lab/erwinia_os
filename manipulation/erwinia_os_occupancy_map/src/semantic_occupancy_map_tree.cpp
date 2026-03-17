#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <octomap_msgs/conversions.h>

namespace octomap
{

    // =======================
    // SemanticOcTreeNode
    // =======================

    SemanticOcTreeNode *SemanticOcTreeNode::createChild(unsigned int i)
    {
        if (!children)
        {
            allocChildren();
        }
        if (!children[i])
        {
            children[i] = new SemanticOcTreeNode();
        }
        return static_cast<SemanticOcTreeNode *>(children[i]);
    }

    SemanticOcTreeNode *SemanticOcTreeNode::getChild(unsigned int i)
    {
        if (!children)
            return nullptr;
        return static_cast<SemanticOcTreeNode *>(children[i]);
    }

    const SemanticOcTreeNode *SemanticOcTreeNode::getChild(unsigned int i) const
    {
        if (!children)
            return nullptr;
        return static_cast<const SemanticOcTreeNode *>(children[i]);
    }

    void SemanticOcTreeNode::expandNode()
    {
        if (!children)
        {
            allocChildren();
        }
        for (unsigned int i = 0; i < 8; ++i)
        {
            if (!children[i])
            {
                children[i] = new SemanticOcTreeNode();
            }
            auto *c = static_cast<SemanticOcTreeNode *>(children[i]);
            c->setLogOdds(this->getLogOdds());
            c->class_id_ = this->class_id_;
            c->confidence_ = this->confidence_;
        }
    }

    std::pair<int32_t, float> SemanticOcTreeNode::getBestChildSemantic() const
    {
        int32_t best_cls = 0;
        float best_conf = 0.0f;

        if (!children)
            return {best_cls, best_conf};

        for (unsigned int i = 0; i < 8; ++i)
        {
            const auto *c = getChild(i);
            if (!c)
                continue;
            if (c->confidence_ > best_conf)
            {
                best_conf = c->confidence_;
                best_cls = c->class_id_;
            }
        }
        return {best_cls, best_conf};
    }

    void SemanticOcTreeNode::updateSemanticChildren()
    {
        auto [best_cls, best_conf] = getBestChildSemantic();
        class_id_ = best_cls;
        confidence_ = best_conf;
    }

    // Only called by full tree serialization
    std::istream &SemanticOcTreeNode::readData(std::istream &s)
    {
        // Read occupancy value (inherited from OcTreeDataNode<float>)
        s.read(reinterpret_cast<char *>(&value), sizeof(value));
        // Read semantic payload
        s.read(reinterpret_cast<char *>(&class_id_), sizeof(class_id_));
        s.read(reinterpret_cast<char *>(&confidence_), sizeof(confidence_));
        return s;
    }

    // Only called by full tree serialization
    std::ostream &SemanticOcTreeNode::writeData(std::ostream &s) const
    {
        // Write occupancy value (inherited from OcTreeDataNode<float>)
        s.write(reinterpret_cast<const char *>(&value), sizeof(value));
        // Write semantic payload
        s.write(reinterpret_cast<const char *>(&class_id_), sizeof(class_id_));
        s.write(reinterpret_cast<const char *>(&confidence_), sizeof(confidence_));
        return s;
    }

    // =======================
    // SemanticOcTree
    // =======================

    SemanticOcTree::StaticMemberInitializer SemanticOcTree::semanticOcTreeMemberInit_;

    SemanticOcTree::StaticMemberInitializer::StaticMemberInitializer()
    {
        // Register prototype once for AbstractOcTree factory (.ot files)
        SemanticOcTree *tree = new SemanticOcTree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
    }

    SemanticOcTree::SemanticOcTree(double resolution)
        : OccupancyOcTreeBase<SemanticOcTreeNode>(resolution)
    {
        semanticOcTreeMemberInit_.ensureLinking();
    }

    void SemanticOcTree::insertPointCloud(const Pointcloud& scan,
                                         const point3d& sensor_origin,
                                         double maxrange,
                                         bool lazy_eval,
                                         bool discretize)
    {
        KeySet free_cells, occupied_cells;
        
        // Handle discretization same as OctoMap base class
        if (discretize) {
            Pointcloud discretePC;
            discretePC.reserve(scan.size());
            KeySet endpoints;
            
            for (size_t i = 0; i < scan.size(); ++i) {
                OcTreeKey k = this->coordToKey(scan[i]);
                std::pair<KeySet::iterator, bool> ret = endpoints.insert(k);
                if (ret.second) { // insertion took place => k was not in set
                    discretePC.push_back(this->keyToCoord(k));
                }
            }
            
            // Recursively call with discretized pointcloud
            insertPointCloud(discretePC, sensor_origin, maxrange, lazy_eval, false);
            return;
        }
        
        // Main update logic (matches OctoMap structure)
#ifdef _OPENMP
        omp_set_num_threads(this->keyrays.size());
        #pragma omp parallel for schedule(guided)
#endif
        for (int i = 0; i < (int)scan.size(); ++i) 
        {
            const point3d& p = scan[i];
            unsigned threadIdx = 0;
#ifdef _OPENMP
            threadIdx = omp_get_thread_num();
#endif
            KeyRay* keyray = &(this->keyrays.at(threadIdx));
            
            if (!use_bbx_limit) 
            { 
                // No BBX specified - use default OctoMap behavior
                if ((maxrange < 0.0) || ((p - sensor_origin).norm() <= maxrange)) 
                { 
                    // is not maxrange meas.
                    // free cells
                    if (this->computeRayKeys(sensor_origin, p, *keyray)) {
#ifdef _OPENMP
                        #pragma omp critical (free_insert)
#endif
                        {
                            free_cells.insert(keyray->begin(), keyray->end());
                        }
                    }
                    // occupied endpoint
                    OcTreeKey key;
                    if (this->coordToKeyChecked(p, key)) {
#ifdef _OPENMP
                        #pragma omp critical (occupied_insert)
#endif
                        {
                            occupied_cells.insert(key);
                        }
                    }
                } 
                else 
                { 
                    // user set a maxrange and length is above
                    point3d direction = (p - sensor_origin).normalized();
                    point3d new_end = sensor_origin + direction * (float)maxrange;
                    if (this->computeRayKeys(sensor_origin, new_end, *keyray)) {
#ifdef _OPENMP
                        #pragma omp critical (free_insert)
#endif
                        {
                            free_cells.insert(keyray->begin(), keyray->end());
                        }
                    }
                }
            } 
            else 
            { 
                // BBX was set - use FIXED behavior
                // endpoint in bbx and not maxrange?
                if (inBBX(p) && ((maxrange < 0.0) || ((p - sensor_origin).norm() <= maxrange))) 
                {
                    // occupied endpoint
                    OcTreeKey key;
                    if (this->coordToKeyChecked(p, key)) {
#ifdef _OPENMP
                        #pragma omp critical (occupied_insert)
#endif
                        {
                            occupied_cells.insert(key);
                        }
                    }
                    
                    // update freespace - FIXED: iterate forward, allow entering BBX
                    if (this->computeRayKeys(sensor_origin, p, *keyray)) 
                    {
                        bool was_inside_bbx = false;
                        
                        // Iterate FORWARD from origin to endpoint
                        for (KeyRay::iterator it = keyray->begin(); 
                             it != keyray->end(); 
                             ++it) 
                        {
                            if (inBBX(*it)) {
#ifdef _OPENMP
                                #pragma omp critical (free_insert)
#endif
                                {
                                    free_cells.insert(*it);
                                }
                                was_inside_bbx = true;
                            }
                            else if (was_inside_bbx) {
                                // Was inside, now outside - break on EXIT
                                break;
                            }
                            // else: still approaching bbox, continue
                        }
                    }
                }
                else
                {
                    // Endpoint is outside BBX or beyond maxrange
                    // Still need to mark free space inside BBX along the ray
                    point3d ray_endpoint = p;
                    
                    // If beyond maxrange, truncate to maxrange
                    if (maxrange > 0.0 && ((p - sensor_origin).norm() > maxrange)) {
                        point3d direction = (p - sensor_origin).normalized();
                        ray_endpoint = sensor_origin + direction * (float)maxrange;
                    }
                    
                    // Compute ray and add free voxels inside BBX
                    if (this->computeRayKeys(sensor_origin, ray_endpoint, *keyray)) 
                    {
                        bool was_inside_bbx = false;
                        
                        // Iterate FORWARD from origin toward endpoint
                        for (KeyRay::iterator it = keyray->begin(); 
                             it != keyray->end(); 
                             ++it) 
                        {
                            if (inBBX(*it)) {
#ifdef _OPENMP
                                #pragma omp critical (free_insert)
#endif
                                {
                                    free_cells.insert(*it);
                                }
                                was_inside_bbx = true;
                            }
                            else if (was_inside_bbx) {
                                // Was inside, now outside - break on EXIT
                                break;
                            }
                            // else: still approaching bbox, continue
                        }
                    }
                }
            }
        }
        
        // prefer occupied cells over free ones (and make sets disjunct)
        for (KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ) 
        {
            if (occupied_cells.find(*it) != occupied_cells.end()) {
                it = free_cells.erase(it);
            } else {
                ++it;
            }
        }
        
        // insert data into tree
        for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
            this->updateNode(*it, false, lazy_eval);
        }
        for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
            this->updateNode(*it, true, lazy_eval);
        }
    }

    SemanticOcTreeNode *SemanticOcTree::integrateNodeSemantic(const OcTreeKey &key,
                                                              int32_t class_id,
                                                              float confidence,
                                                              bool require_node_exist,
                                                              bool lazy_eval,
                                                              float confidence_boost,
                                                              float mismatch_penalty)
    {
        if (confidence <= 0.0f)
            return nullptr;

        SemanticOcTreeNode *node = this->search(key);

        if (!node)
        {
            if (require_node_exist)
            {
                return nullptr;
            }
            // Create node as occupied (semantic implies a hit / endpoint)
            node = this->updateNode(key, true /*occupied*/, lazy_eval);
            if (!node)
                return nullptr;
        }

        node->fuseSemanticMax(class_id, confidence, confidence_boost, mismatch_penalty);
        return node;
    }

    void SemanticOcTree::updateInnerOccupancySemantic()
    {
        // First: call the base behavior (non-virtual) to update occupancy correctly
        OccupancyOcTreeBase<SemanticOcTreeNode>::updateInnerOccupancy();

        // Then: propagate semantics upward (best child)
        updateInnerOccupancySemanticRecurs(this->root, 0);
    }

    void SemanticOcTree::updateInnerOccupancySemanticRecurs(SemanticOcTreeNode *node, unsigned int /*depth*/)
    {
        if (!node)
            return;
        if (!this->nodeHasChildren(node))
            return;

        for (unsigned int i = 0; i < 8; ++i)
        {
            SemanticOcTreeNode *c = node->getChild(i);
            if (c)
                updateInnerOccupancySemanticRecurs(c, 0);
        }
        node->updateSemanticChildren();
    }

} // namespace octomap

namespace erwinia_os_occupancy_map
{

    static bool ends_with(const std::string &s, const std::string &suffix)
    {
        if (suffix.size() > s.size())
            return false;
        return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
    }

    SemanticOccupancyMapTree::SemanticOccupancyMapTree(double resolution)
        : octomap::SemanticOcTree(resolution)
    {
    }

    SemanticOccupancyMapTree::SemanticOccupancyMapTree(const std::string &filename)
        : octomap::SemanticOcTree(0.1)
    {
        if (ends_with(filename, ".bt"))
        {
            if (!this->readBinary(filename))
            {
                throw std::runtime_error("SemanticOccupancyMapTree: failed to read .bt file: " + filename);
            }
            return;
        }

        if (ends_with(filename, ".ot"))
        {
            std::unique_ptr<octomap::AbstractOcTree> abs(octomap::AbstractOcTree::read(filename));
            if (!abs)
            {
                throw std::runtime_error("SemanticOccupancyMapTree: failed to read .ot file: " + filename);
            }

            auto *sem = dynamic_cast<octomap::SemanticOcTree *>(abs.get());
            if (!sem)
            {
                throw std::runtime_error("SemanticOccupancyMapTree: .ot file is not SemanticOcTree: " + filename);
            }

            // Reconstruct our base subobject using copy constructor (operator= is deleted).
            // We are in the derived constructor, so this is the clean way to "load into this".
            this->~SemanticOccupancyMapTree();
            new (this) SemanticOccupancyMapTree(*sem); // uses copy ctor we define below
            return;
        }

        throw std::runtime_error("SemanticOccupancyMapTree: unsupported file extension: " + filename);
    }

    SemanticOccupancyMapTree::SemanticOccupancyMapTree(const octomap::SemanticOcTree &rhs)
        : octomap::SemanticOcTree(rhs) // OccupancyOcTreeBase has copy ctor, so this works
    {
    }

    void SemanticOccupancyMapTree::lockRead() { tree_mutex_.lock_shared(); }
    void SemanticOccupancyMapTree::unlockRead() { tree_mutex_.unlock_shared(); }
    void SemanticOccupancyMapTree::lockWrite() { tree_mutex_.lock(); }
    void SemanticOccupancyMapTree::unlockWrite() { tree_mutex_.unlock(); }

    SemanticOccupancyMapTree::ReadLock SemanticOccupancyMapTree::reading()
    {
        return ReadLock(tree_mutex_);
    }

    SemanticOccupancyMapTree::WriteLock SemanticOccupancyMapTree::writing()
    {
        return WriteLock(tree_mutex_);
    }

    void SemanticOccupancyMapTree::triggerUpdateCallback()
    {
        if (update_callback_)
            update_callback_();
    }

    void SemanticOccupancyMapTree::setUpdateCallback(const std::function<void()> &callback)
    {
        update_callback_ = callback;
    }

    size_t SemanticOccupancyMapTree::getNumOccupiedNodes() const
    {
        size_t count = 0;
        for (auto it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            if (this->isNodeOccupied(*it))
                ++count;
        }
        return count;
    }

    size_t SemanticOccupancyMapTree::getNumFreeNodes() const
    {
        size_t count = 0;
        for (auto it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            if (!this->isNodeOccupied(*it))
                ++count;
        }
        return count;
    }

    size_t SemanticOccupancyMapTree::memoryUsage() const
    {
        // Simple approximation; OctoMap has internal memoryUsage() in some builds but not always exposed
        return this->size() * sizeof(octomap::SemanticOcTreeNode);
    }

    octomap::AbstractOcTree* semanticMsgToMap(const octomap_msgs::msg::Octomap& msg)
    {
        // Check if it's a semantic octree
        if (msg.id == "SemanticOcTree")
        {
            octomap::SemanticOcTree* tree = new octomap::SemanticOcTree(msg.resolution);
            
            std::stringstream datastream;
            if (msg.data.size() > 0)
            {
                datastream.write((const char*)&msg.data[0], msg.data.size());
                
                if (msg.binary)
                {
                    tree->readBinaryData(datastream);
                }
                else
                {
                    tree->readData(datastream);
                }
            }
            return tree;
        }
        
        // Fall back to standard octomap_msgs conversion for OcTree and ColorOcTree
        return octomap_msgs::msgToMap(msg);
    }

} // namespace erwinia_os_occupancy_map
