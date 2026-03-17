/**
 * @file occupancy_map_tree.cpp
 * @brief Implementation of thread-safe octomap wrapper
 */

#include "erwinia_os_occupancy_map/occupancy_map_tree.hpp"
#include <octomap/octomap.h>

namespace erwinia_os_occupancy_map
{

    OccupancyMapTree::OccupancyMapTree(double resolution)
        : octomap::OcTree(resolution)
    {
    }

    OccupancyMapTree::OccupancyMapTree(const std::string &filename)
        : octomap::OcTree(filename)
    {
    }

    // ============================================================================
    // OVERRIDE: Fixed bounding box behavior for free space
    // ============================================================================

    void OccupancyMapTree::insertPointCloud(const octomap::Pointcloud& scan,
                                           const octomap::point3d& sensor_origin,
                                           double maxrange,
                                           bool lazy_eval,
                                           bool discretize)
    {
        octomap::KeySet free_cells, occupied_cells;
        
        // Handle discretization same as OctoMap base class
        if (discretize) {
            octomap::Pointcloud discretePC;
            discretePC.reserve(scan.size());
            octomap::KeySet endpoints;
            
            for (size_t i = 0; i < scan.size(); ++i) {
                octomap::OcTreeKey k = this->coordToKey(scan[i]);
                std::pair<octomap::KeySet::iterator, bool> ret = endpoints.insert(k);
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
            const octomap::point3d& p = scan[i];
            unsigned threadIdx = 0;
#ifdef _OPENMP
            threadIdx = omp_get_thread_num();
#endif
            octomap::KeyRay* keyray = &(this->keyrays.at(threadIdx));
            
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
                    octomap::OcTreeKey key;
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
                    octomap::point3d direction = (p - sensor_origin).normalized();
                    octomap::point3d new_end = sensor_origin + direction * (float)maxrange;
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
                    octomap::OcTreeKey key;
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
                        for (octomap::KeyRay::iterator it = keyray->begin(); 
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
                    octomap::point3d ray_endpoint = p;
                    
                    // If beyond maxrange, truncate to maxrange
                    if (maxrange > 0.0 && ((p - sensor_origin).norm() > maxrange)) {
                        octomap::point3d direction = (p - sensor_origin).normalized();
                        ray_endpoint = sensor_origin + direction * (float)maxrange;
                    }
                    
                    // Compute ray and add free voxels inside BBX
                    if (this->computeRayKeys(sensor_origin, ray_endpoint, *keyray)) 
                    {
                        bool was_inside_bbx = false;
                        
                        // Iterate FORWARD from origin toward endpoint
                        for (octomap::KeyRay::iterator it = keyray->begin(); 
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
        for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ) 
        {
            if (occupied_cells.find(*it) != occupied_cells.end()) {
                it = free_cells.erase(it);
            } else {
                ++it;
            }
        }
        
        // insert data into tree
        for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
            this->updateNode(*it, false, lazy_eval);
        }
        for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
            this->updateNode(*it, true, lazy_eval);
        }
    }

    // ============================================================================
    // Thread-safe locking
    // ============================================================================

    void OccupancyMapTree::lockRead()
    {
        tree_mutex_.lock_shared();
    }

    void OccupancyMapTree::unlockRead()
    {
        tree_mutex_.unlock_shared();
    }

    void OccupancyMapTree::lockWrite()
    {
        tree_mutex_.lock();
    }

    void OccupancyMapTree::unlockWrite()
    {
        tree_mutex_.unlock();
    }

    OccupancyMapTree::ReadLock OccupancyMapTree::reading()
    {
        return ReadLock(tree_mutex_);
    }

    OccupancyMapTree::WriteLock OccupancyMapTree::writing()
    {
        return WriteLock(tree_mutex_);
    }

    // ============================================================================
    // Update callbacks
    // ============================================================================

    void OccupancyMapTree::triggerUpdateCallback()
    {
        if (update_callback_)
        {
            update_callback_();
        }
    }

    void OccupancyMapTree::setUpdateCallback(const std::function<void()> &callback)
    {
        update_callback_ = callback;
    }

    // ============================================================================
    // Statistics and queries
    // ============================================================================

    size_t OccupancyMapTree::getNumOccupiedNodes() const
    {
        size_t count = 0;
        for (auto it = this->begin_leafs(); it != this->end_leafs(); ++it)
        {
            if (this->isNodeOccupied(*it))
            {
                count++;
            }
        }
        return count;
    }

    size_t OccupancyMapTree::getNumFreeNodes() const
    {
        size_t count = 0;
        for (auto it = this->begin_leafs(); it != this->end_leafs(); ++it)
        {
            if (!this->isNodeOccupied(*it))
            {
                count++;
            }
        }
        return count;
    }

    size_t OccupancyMapTree::memoryUsage() const
    {
        return this->memoryUsage();
    }

} // namespace erwinia_os_occupancy_map
