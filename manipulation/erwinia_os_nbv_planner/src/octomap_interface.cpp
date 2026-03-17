#include "erwinia_os_nbv_planner/octomap_interface.hpp"
#include "erwinia_os_occupancy_map/semantic_occupancy_map_tree.hpp"

#include <octomap_msgs/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>
#include <random>
#include <numeric>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace erwinia_os_nbv_planner
{

    OctoMapInterface::OctoMapInterface(const rclcpp::Node::SharedPtr &node,
                                       const std::string &octomap_topic,
                                       bool transient_local) : node_(node)
    {
        rclcpp::QoS qos(1);
        if (transient_local)
            qos.transient_local();

        sub_ = node_->create_subscription<erwinia_os_occupancy_map::msg::CustomOctomap>(
            octomap_topic, qos,
            std::bind(&OctoMapInterface::onOctomap, this, std::placeholders::_1));

        // Initialize TF2 for coordinate transformations
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(node_->get_logger(), "OctoMapInterface subscribing to %s", octomap_topic.c_str());
    }

    void OctoMapInterface::onOctomap(const erwinia_os_occupancy_map::msg::CustomOctomap::SharedPtr msg)
    {
        // Use custom conversion that handles semantic trees
        std::unique_ptr<octomap::AbstractOcTree> abs(erwinia_os_occupancy_map::semanticMsgToMap(msg->octomap));
        if (!abs)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to deserialize octomap");
            return;
        }

        // Determine tree type from the message ID
        OctoMapType new_type = OctoMapType::STANDARD;
        if (msg->octomap.id == "SemanticOcTree")
        {
            new_type = OctoMapType::SEMANTIC;
        }

        std::unique_lock lk(mtx_);

        // Try to cast to appropriate type
        if (new_type == OctoMapType::SEMANTIC)
        {
            auto *sem_tree = dynamic_cast<octomap::SemanticOcTree *>(abs.get());
            if (sem_tree)
            {
                tree_ = std::shared_ptr<octomap::SemanticOcTree>(sem_tree);
                abs.release(); // Don't double-delete
                tree_type_ = OctoMapType::SEMANTIC;
                RCLCPP_INFO(node_->get_logger(), "Loaded semantic octomap");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Expected semantic tree but cast failed");
                return;
            }
        }
        else
        {
            auto *std_tree = dynamic_cast<octomap::OcTree *>(abs.get());
            if (std_tree)
            {
                tree_ = std::shared_ptr<octomap::OcTree>(std_tree);
                abs.release();
                tree_type_ = OctoMapType::STANDARD;
                RCLCPP_INFO(node_->get_logger(), "Loaded standard octomap");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Expected standard tree but cast failed");
                return;
            }
        }

        resolution_ = msg->octomap.resolution;
        last_update_time_ = node_->now();
        octomap_frame_id_ = msg->header.frame_id;

        // Use bounding box from message if provided
        if (msg->has_bounding_box)
        {
            bbox_min_ = octomap::point3d(msg->bbx_min.x, msg->bbx_min.y, msg->bbx_min.z);
            bbox_max_ = octomap::point3d(msg->bbx_max.x, msg->bbx_max.y, msg->bbx_max.z);
            has_valid_bbox_ = true;
            RCLCPP_DEBUG(node_->get_logger(), "Octomap received: type=%s, resolution=%.4f, bbox=[%.2f,%.2f,%.2f] to [%.2f,%.2f,%.2f]",
                         msg->octomap.id.c_str(), resolution_,
                         bbox_min_.x(), bbox_min_.y(), bbox_min_.z(),
                         bbox_max_.x(), bbox_max_.y(), bbox_max_.z());
        }
        else
        {
            has_valid_bbox_ = false;
            RCLCPP_DEBUG(node_->get_logger(), "Octomap received: type=%s, resolution=%.4f (no bounding box)",
                         msg->octomap.id.c_str(), resolution_);
        }
    }

    std::shared_ptr<octomap::AbstractOcTree> OctoMapInterface::getTreeSnapshot() const
    {
        std::shared_lock lk(mtx_);
        return std::visit([](auto &&tree) -> std::shared_ptr<octomap::AbstractOcTree>
                          { return tree; }, tree_);
    }

    bool OctoMapInterface::getVoxelSemantic(const octomap::point3d &point,
                                            int32_t &class_id,
                                            float &confidence) const
    {
        auto sem_tree = getSemanticTree();
        if (!sem_tree)
            return false;

        octomap::SemanticOcTreeNode *node = sem_tree->search(point);
        if (!node)
            return false;

        class_id = node->getClassId();
        confidence = node->getConfidence();
        return true;
    }

    std::vector<octomap::point3d> OctoMapInterface::findFrontiersByClass(
        int32_t class_id,
        float min_confidence,
        int min_unknown_neighbors,
        bool use_bbox) const
    {
        std::vector<octomap::point3d> filtered;

        auto sem_tree = getSemanticTree();
        if (!sem_tree)
        {
            RCLCPP_WARN(node_->get_logger(), "findFrontiersByClass called on non-semantic tree");
            return filtered;
        }

        // Get all frontiers first
        auto frontiers = findFrontiers(min_unknown_neighbors, use_bbox);

        // Filter by semantic class
        for (const auto &pt : frontiers)
        {
            octomap::SemanticOcTreeNode *node = sem_tree->search(pt);
            if (node &&
                node->getClassId() == class_id &&
                node->getConfidence() >= min_confidence)
            {
                filtered.push_back(pt);
            }
        }

        return filtered;
    }

    std::map<int32_t, size_t> OctoMapInterface::getSemanticClassCounts() const
    {
        std::map<int32_t, size_t> class_counts;

        auto sem_tree = getSemanticTree();
        if (!sem_tree)
        {
            return class_counts; // Return empty map for non-semantic trees
        }

        // Iterate through all leaf nodes and count occupied voxels by class
        for (auto it = sem_tree->begin_leafs(); it != sem_tree->end_leafs(); ++it)
        {
            if (sem_tree->isNodeOccupied(*it))
            {
                int32_t class_id = it->getClassId();
                class_counts[class_id]++;
            }
        }

        return class_counts;
    }

    std::vector<octomap::point3d> OctoMapInterface::findFrontiers(int min_unknown_neighbors,
                                                                  bool use_bbox) const
    {
        std::vector<octomap::point3d> frontiers;

        std::shared_lock lk(mtx_);

        double res = 0.0;

        // Check tree type and get resolution
        if (tree_type_ == OctoMapType::STANDARD)
        {
            auto std_tree = std::get<std::shared_ptr<octomap::OcTree>>(tree_);
            if (!std_tree)
                return frontiers;
            res = std_tree->getResolution();
        }
        else if (tree_type_ == OctoMapType::SEMANTIC)
        {
            auto sem_tree = std::get<std::shared_ptr<octomap::SemanticOcTree>>(tree_);
            if (!sem_tree)
                return frontiers;
            res = sem_tree->getResolution();
        }
        else
        {
            return frontiers;
        }

        // If using bbox, check if we have valid bounds
        if (use_bbox && !has_valid_bbox_)
        {
            RCLCPP_WARN(node_->get_logger(), "findFrontiers: use_bbox=true but no valid bounding box available");
            return frontiers;
        }

        // Lambda to check if point is inside bbox
        auto is_in_bbox = [&](const octomap::point3d &p) -> bool
        {
            if (!use_bbox)
                return true;
            return p.x() >= bbox_min_.x() && p.x() <= bbox_max_.x() &&
                   p.y() >= bbox_min_.y() && p.y() <= bbox_max_.y() &&
                   p.z() >= bbox_min_.z() && p.z() <= bbox_max_.z();
        };

        // Use std::visit to handle both tree types
        std::visit([&](auto &&tree)
                   {
            if (!tree) return;
            
            // Lambdas for checking unknown neighbors
            auto is_unknown = [&](const octomap::point3d &p) -> bool
            {
                // If using bbox, don't count voxels outside bbox as unknown
                if (use_bbox && !is_in_bbox(p))
                {
                    return false;
                }
                return tree->search(p) == nullptr;
            };

            // Count unknown neighbors in 6-neighborhood
            auto count_unknown_6 = [&](const octomap::point3d &v) -> int
            {
                int unknown = 0;
                unknown += is_unknown(v + octomap::point3d(res, 0, 0));
                unknown += is_unknown(v + octomap::point3d(-res, 0, 0));
                unknown += is_unknown(v + octomap::point3d(0, res, 0));
                unknown += is_unknown(v + octomap::point3d(0, -res, 0));
                unknown += is_unknown(v + octomap::point3d(0, 0, res));
                unknown += is_unknown(v + octomap::point3d(0, 0, -res));
                return unknown;
            };

            // Search for frontiers (use bounding box if specified)
            if (!use_bbox)
            {
                for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
                {
                    if (tree->isNodeOccupied(*it))
                        continue; // only free
                    const auto v = it.getCoordinate();
                    if (count_unknown_6(v) >= min_unknown_neighbors)
                        frontiers.push_back(v);
                }
            }
            else
            {
                for (auto it = tree->begin_leafs_bbx(bbox_min_, bbox_max_); it != tree->end_leafs_bbx(); ++it)
                {
                    if (tree->isNodeOccupied(*it))
                        continue;
                    const auto v = it.getCoordinate();
                    if (count_unknown_6(v) >= min_unknown_neighbors)
                        frontiers.push_back(v);
                }
            } }, tree_);

        return frontiers;
    }

    std::vector<octomap::point3d> OctoMapInterface::getUncertainVoxels(
        float max_confidence,
        bool use_bbox,
        bool count_background) const
    {
        std::vector<octomap::point3d> voxels;

        auto sem_tree = getSemanticTree();
        if (!sem_tree)
        {
            RCLCPP_WARN(node_->get_logger(), "getUncertainVoxels called on non-semantic tree");
            return voxels;
        }

        // Clamp threshold defensively
        if (std::isnan(max_confidence) || std::isinf(max_confidence))
            max_confidence = 0.5f;
        max_confidence = std::min(1.0f, std::max(0.0f, max_confidence));

        // BBox availability
        if (!hasBoundingBox() && use_bbox)
        {
            RCLCPP_WARN(node_->get_logger(), "getUncertainVoxels: use_bbox=true but no valid bounding box available");
            return voxels;
        }

        auto is_in_bbox = [&](const octomap::point3d &p) -> bool
        {
            if (!use_bbox) return true;
            return p.x() >= bbox_min_.x() && p.x() <= bbox_max_.x() &&
                p.y() >= bbox_min_.y() && p.y() <= bbox_max_.y() &&
                p.z() >= bbox_min_.z() && p.z() <= bbox_max_.z();
        };

        // Iterate leaf nodes (optionally within bbox)
        if (!use_bbox)
        {
            for (auto it = sem_tree->begin_leafs(); it != sem_tree->end_leafs(); ++it)
            {
                if (!sem_tree->isNodeOccupied(*it))
                    continue;

                const auto *node = &(*it);

                if (!count_background && !node->isSemanticSet())
                    continue;

                float conf = node->getConfidence();
                if (std::isnan(conf) || std::isinf(conf)) conf = 0.0f;
                conf = std::min(1.0f, std::max(0.0f, conf));

                if (conf <= max_confidence)
                    voxels.push_back(it.getCoordinate()); // voxel center
            }
        }
        else
        {
            for (auto it = sem_tree->begin_leafs_bbx(bbox_min_, bbox_max_); it != sem_tree->end_leafs_bbx(); ++it)
            {
                if (!sem_tree->isNodeOccupied(*it))
                    continue;

                const auto *node = &(*it);

                if (!count_background && !node->isSemanticSet())
                    continue;

                float conf = node->getConfidence();
                if (std::isnan(conf) || std::isinf(conf)) conf = 0.0f;
                conf = std::min(1.0f, std::max(0.0f, conf));

                // Note: begin_leafs_bbx should already be inside bbox, but keep it robust:
                const auto p = it.getCoordinate();
                if (!is_in_bbox(p))
                    continue;

                if (conf <= max_confidence)
                    voxels.push_back(p);
            }
        }

        return voxels;
    }

    static inline double sqdist(const octomap::point3d &a, const octomap::point3d &b)
    {
        const double dx = a.x() - b.x();
        const double dy = a.y() - b.y();
        const double dz = a.z() - b.z();
        return dx * dx + dy * dy + dz * dz;
    }

    std::vector<Cluster> OctoMapInterface::kmeansCluster(const std::vector<octomap::point3d> &points,
                                                         int n_clusters,
                                                         int max_iters,
                                                         double tol) const
    {
        std::vector<Cluster> clusters;
        const int N = static_cast<int>(points.size());
        if (N == 0)
            return clusters;

        // Heuristic: ~1 cluster per 50 points
        if (n_clusters <= 0)
            n_clusters = std::max(1, N / 50);

        const int K = std::min(n_clusters, N);

        // Working data structures for k-means algorithm
        std::vector<int> labels(N, -1);
        std::vector<octomap::point3d> centers(K);
        std::vector<int> sizes(K, 0);

        // K-means++ initialization for faster convergence
        std::mt19937 rng(42); // Deterministic seed
        centers[0] = points[rng() % N];

        for (int k = 1; k < K; ++k)
        {
            std::vector<double> min_dists(N, std::numeric_limits<double>::infinity());

            // Update min distances to existing centers
            for (int i = 0; i < N; ++i)
            {
                double d = sqdist(points[i], centers[k - 1]);
                min_dists[i] = std::min(min_dists[i], d);
            }

            // Choose next center with probability proportional to squared distance
            double sum_dists = std::accumulate(min_dists.begin(), min_dists.end(), 0.0);
            std::uniform_real_distribution<double> dist(0.0, sum_dists);
            double target = dist(rng);

            double cumsum = 0.0;
            for (int i = 0; i < N; ++i)
            {
                cumsum += min_dists[i];
                if (cumsum >= target)
                {
                    centers[k] = points[i];
                    break;
                }
            }
        }

        std::vector<octomap::point3d> new_centers(K);
        std::vector<double> sum_x(K), sum_y(K), sum_z(K);

        // K-means iteration
        for (int iter = 0; iter < max_iters; ++iter)
        {
            // Reset accumulators
            std::fill(sizes.begin(), sizes.end(), 0);
            std::fill(sum_x.begin(), sum_x.end(), 0.0);
            std::fill(sum_y.begin(), sum_y.end(), 0.0);
            std::fill(sum_z.begin(), sum_z.end(), 0.0);

            // Assignment step: assign each point to nearest center
            for (int i = 0; i < N; ++i)
            {
                int best_k = 0;
                double best_d = sqdist(points[i], centers[0]);

                for (int k = 1; k < K; ++k)
                {
                    double d = sqdist(points[i], centers[k]);
                    if (d < best_d)
                    {
                        best_d = d;
                        best_k = k;
                    }
                }

                labels[i] = best_k;
                sizes[best_k]++;
                sum_x[best_k] += points[i].x();
                sum_y[best_k] += points[i].y();
                sum_z[best_k] += points[i].z();
            }

            // Update step: recompute centers and check convergence
            double max_shift = 0.0;
            for (int k = 0; k < K; ++k)
            {
                if (sizes[k] > 0)
                {
                    const double inv = 1.0 / static_cast<double>(sizes[k]);
                    octomap::point3d c(sum_x[k] * inv, sum_y[k] * inv, sum_z[k] * inv);
                    max_shift = std::max(max_shift, std::sqrt(sqdist(c, centers[k])));
                    centers[k] = c;
                }
            }

            if (max_shift < tol)
                break;
        }

        // Build output: create Cluster objects with their assigned points
        clusters.resize(K);
        for (int k = 0; k < K; ++k)
        {
            clusters[k].label = k;
            clusters[k].center = centers[k];
            clusters[k].size = sizes[k];
            clusters[k].points.reserve(sizes[k]);
        }

        // Populate points for each cluster
        for (int i = 0; i < N; ++i)
        {
            clusters[labels[i]].points.push_back(points[i]);
        }

        return clusters;
    }

    bool OctoMapInterface::isVoxelOccupied(const octomap::point3d &point) const
    {
        std::shared_lock lk(mtx_);

        return std::visit([&point](auto &&tree) -> bool
                          {
            if (!tree)
                return false;
            auto *node = tree->search(point);
            return node != nullptr && tree->isNodeOccupied(node); }, tree_);
    }

    bool OctoMapInterface::searchOctree(const octomap::point3d &point,
                                        octomap::OcTreeNode *&node_out) const
    {
        std::shared_lock lk(mtx_);

        return std::visit([&point, &node_out](auto &&tree) -> bool
                          {
            if (!tree)
            {
                node_out = nullptr;
                return false;
            }
            node_out = tree->search(point);
            return node_out != nullptr; }, tree_);
    }

    int OctoMapInterface::getOctreeDepth() const
    {
        std::shared_lock lk(mtx_);

        return std::visit([](auto &&tree) -> int
                          {
            if (!tree)
                return 0;
            return static_cast<int>(tree->getTreeDepth()); }, tree_);
    }

    bool OctoMapInterface::loadGroundTruthSemantics(const std::string &yaml_file_path)
    {
        try
        {
            RCLCPP_INFO(node_->get_logger(), "Loading ground truth from: %s", yaml_file_path.c_str());

            YAML::Node config = YAML::LoadFile(yaml_file_path);

            if (!config["markers"])
            {
                RCLCPP_ERROR(node_->get_logger(), "YAML file does not contain 'markers' field");
                return false;
            }

            // Read frame_id if present
            if (config["frame_id"])
            {
                gt_frame_id_ = config["frame_id"].as<std::string>();
                RCLCPP_INFO(node_->get_logger(), "Ground truth frame_id: %s", gt_frame_id_.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "No 'frame_id' field in YAML, frame consistency cannot be verified");
                gt_frame_id_ = "";
            }

            gt_points_.clear();
            gt_classes_.clear();
            std::set<int32_t> unique_classes;

            const YAML::Node &markers = config["markers"];
            for (const auto &marker_node : markers)
            {
                SemanticPoint point;

                if (!marker_node["id"] || !marker_node["class_id"] || !marker_node["position"])
                {
                    RCLCPP_WARN(node_->get_logger(), "Skipping marker with missing fields");
                    continue;
                }

                point.id = marker_node["id"].as<int>();
                point.class_id = marker_node["class_id"].as<int32_t>();

                const YAML::Node &pos = marker_node["position"];
                point.position = octomap::point3d(
                    pos["x"].as<double>(),
                    pos["y"].as<double>(),
                    pos["z"].as<double>());

                gt_points_.push_back(point);
                unique_classes.insert(point.class_id);

                RCLCPP_DEBUG(node_->get_logger(),
                             "Loaded marker %d (class %d) at (%.3f, %.3f, %.3f)",
                             point.id, point.class_id,
                             point.position.x(), point.position.y(), point.position.z());
            }

            // Convert set to vector for gt_classes_
            gt_classes_.assign(unique_classes.begin(), unique_classes.end());

            RCLCPP_INFO(node_->get_logger(), "Loaded %zu ground truth markers with %zu unique classes", 
                        gt_points_.size(), gt_classes_.size());
            return true;
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error loading ground truth: %s", e.what());
            return false;
        }
    }

    std::vector<Cluster> OctoMapInterface::clusterSemanticVoxels(bool verbose) const
    {
        std::vector<Cluster> clusters;

        auto sem_tree = getSemanticTree();
        if (!sem_tree) return clusters;

        // Group leaf keys by class
        std::map<int32_t, std::vector<octomap::OcTreeKey>> keys_by_class;

        for (auto it = sem_tree->begin_leafs(); it != sem_tree->end_leafs(); ++it)
        {
            if (!sem_tree->isNodeOccupied(*it)) continue;

            const int32_t class_id = it->getClassId();
            if (class_id == -1) continue;

            // Get discrete key for this leaf voxel
            octomap::OcTreeKey key;
            sem_tree->coordToKeyChecked(it.getCoordinate(), key);
            keys_by_class[class_id].push_back(key);
        }

        // Hash for OcTreeKey (3 uint16-ish ints)
        struct KeyHash {
            std::size_t operator()(const octomap::OcTreeKey& k) const noexcept {
                // simple combine
                std::size_t h1 = std::hash<unsigned int>{}(k.k[0]);
                std::size_t h2 = std::hash<unsigned int>{}(k.k[1]);
                std::size_t h3 = std::hash<unsigned int>{}(k.k[2]);
                return h1 ^ (h2 << 1) ^ (h3 << 2);
            }
        };
        struct KeyEq {
            bool operator()(const octomap::OcTreeKey& a, const octomap::OcTreeKey& b) const noexcept {
                return a.k[0]==b.k[0] && a.k[1]==b.k[1] && a.k[2]==b.k[2];
            }
        };

        for (const auto& [class_id, keys] : keys_by_class)
        {
            std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> unvisited;
            unvisited.reserve(keys.size() * 2);
            for (const auto& k : keys) unvisited.insert(k);

            while (!unvisited.empty())
            {
                Cluster cluster;
                cluster.class_id = class_id;

                std::queue<octomap::OcTreeKey> q;
                auto start_it = unvisited.begin();
                octomap::OcTreeKey start = *start_it;
                q.push(start);
                unvisited.erase(start_it);

                double sum_x = 0, sum_y = 0, sum_z = 0;

                while (!q.empty())
                {
                    auto cur = q.front(); q.pop();

                    // Convert key back to voxel center
                    octomap::point3d p = sem_tree->keyToCoord(cur);
                    cluster.points.push_back(p);
                    sum_x += p.x(); sum_y += p.y(); sum_z += p.z();

                    // 6-connected neighbors in key space
                    const int dx[6] = {+1,-1, 0, 0, 0, 0};
                    const int dy[6] = { 0, 0,+1,-1, 0, 0};
                    const int dz[6] = { 0, 0, 0, 0,+1,-1};

                    for (int i=0;i<6;i++)
                    {
                        octomap::OcTreeKey nb = cur;
                        nb.k[0] = static_cast<unsigned int>(static_cast<int>(nb.k[0]) + dx[i]);
                        nb.k[1] = static_cast<unsigned int>(static_cast<int>(nb.k[1]) + dy[i]);
                        nb.k[2] = static_cast<unsigned int>(static_cast<int>(nb.k[2]) + dz[i]);

                        auto it = unvisited.find(nb);
                        if (it != unvisited.end())
                        {
                            q.push(nb);
                            unvisited.erase(it);
                        }
                    }
                }

                const size_t n = cluster.points.size();
                if (n > 0)
                {
                    cluster.center = octomap::point3d(sum_x/n, sum_y/n, sum_z/n);
                    cluster.size = static_cast<int>(n);
                    cluster.label = static_cast<int>(clusters.size());
                    clusters.push_back(std::move(cluster));
                }
            }
        }

        if (verbose)
        {
            RCLCPP_INFO(node_->get_logger(), "Found %zu semantic clusters", clusters.size());

            // Log cluster details
            std::map<int32_t, int> clusters_per_class;
            for (const auto &cluster : clusters)
            {
                clusters_per_class[cluster.class_id]++;
            }

            RCLCPP_INFO(node_->get_logger(), "Clusters per class:");
            for (const auto &[class_id, count] : clusters_per_class)
            {
                RCLCPP_INFO(node_->get_logger(), "  Class %d: %d clusters", class_id, count);
            }
        }
        return clusters;
    }

    MatchResult OctoMapInterface::matchClustersToGroundTruth(const std::vector<Cluster> &clusters,
                                                             double threshold_radius,
                                                             bool verbose) const
    {
        MatchResult result;

        if (gt_points_.empty())
        {
            if (verbose)
                RCLCPP_WARN(node_->get_logger(), "No ground truth points loaded for matching");
            // All clusters are unmatched (false positives)
            result.unmatched_clusters = clusters;
            return result;
        }

        // Get octomap frame (need to copy since we're const)
        std::string octomap_frame;
        {
            std::shared_lock lk(mtx_);
            octomap_frame = octomap_frame_id_;
        }

        // Transform GT points to octomap frame if necessary
        std::vector<SemanticPoint> transformed_gt_points = gt_points_;
        
        if (!gt_frame_id_.empty() && !octomap_frame.empty())
        {
            if (gt_frame_id_ != octomap_frame)
            {
                RCLCPP_INFO(node_->get_logger(), 
                    "Transforming ground truth from frame '%s' to octomap frame '%s'",
                    gt_frame_id_.c_str(), octomap_frame.c_str());
                
                try
                {
                    // Get transform from GT frame to octomap frame
                    geometry_msgs::msg::TransformStamped transform_stamped = 
                        tf_buffer_->lookupTransform(octomap_frame, gt_frame_id_, 
                                                   tf2::TimePointZero);
                    
                    // Transform each GT point
                    for (auto& gt_point : transformed_gt_points)
                    {
                        geometry_msgs::msg::PointStamped point_in, point_out;
                        point_in.header.frame_id = gt_frame_id_;
                        point_in.point.x = gt_point.position.x();
                        point_in.point.y = gt_point.position.y();
                        point_in.point.z = gt_point.position.z();
                        
                        tf2::doTransform(point_in, point_out, transform_stamped);
                        
                        gt_point.position = octomap::point3d(
                            point_out.point.x,
                            point_out.point.y,
                            point_out.point.z);
                    }
                    
                    RCLCPP_INFO(node_->get_logger(), 
                        "Successfully transformed %zu ground truth points",
                        transformed_gt_points.size());
                }
                catch (const tf2::TransformException& ex)
                {
                    RCLCPP_ERROR(node_->get_logger(), 
                        "Failed to transform ground truth points: %s. "
                        "Proceeding with original coordinates (may produce incorrect results).",
                        ex.what());
                    // Keep original points if transform fails
                }
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), 
                    "Ground truth and octomap are in same frame '%s'", 
                    gt_frame_id_.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), 
                "Frame IDs not available (GT: '%s', octomap: '%s') - assuming same frame",
                gt_frame_id_.c_str(), octomap_frame.c_str());
        }

        // Warn about any GT points that fall outside the octomap bounding box.
        // These can never produce a match and almost certainly indicate a frame or
        // configuration mistake.
        {
            std::shared_lock lk(mtx_);
            if (has_valid_bbox_)
            {
                for (const auto &gt_pt : transformed_gt_points)
                {
                    const auto &p = gt_pt.position;
                    if (p.x() < bbox_min_.x() || p.x() > bbox_max_.x() ||
                        p.y() < bbox_min_.y() || p.y() > bbox_max_.y() ||
                        p.z() < bbox_min_.z() || p.z() > bbox_max_.z())
                    {
                        const std::string msg = [&] {
                            std::ostringstream oss;
                            oss << "Ground truth point (class " << gt_pt.class_id << ") ["
                                << p.x() << ", " << p.y() << ", " << p.z()
                                << "] is OUTSIDE the octomap bounding box ["
                                << bbox_min_.x() << "," << bbox_min_.y() << "," << bbox_min_.z()
                                << "] -> ["
                                << bbox_max_.x() << "," << bbox_max_.y() << "," << bbox_max_.z()
                                << "]. Check GT frame_id / transform.";
                            return oss.str();
                        }();
                        RCLCPP_FATAL(node_->get_logger(), "%s", msg.c_str());
                        throw std::runtime_error(msg);
                    }
                }
            }
        }

        if (verbose)
        {
            RCLCPP_INFO(node_->get_logger(), "Matching %zu clusters to %zu ground truth points (threshold: %.3f m)",
                        clusters.size(), transformed_gt_points.size(), threshold_radius);
        }

        double threshold_sq = threshold_radius * threshold_radius;
        std::set<size_t> matched_cluster_indices;

        // For each GT point, find all clusters within threshold
        for (size_t j = 0; j < transformed_gt_points.size(); ++j)
        {
            const auto &gt_point = transformed_gt_points[j];

            std::vector<Cluster> matching_class_clusters;
            std::vector<double> matching_class_distances;
            std::vector<Cluster> mismatching_class_clusters;
            std::vector<double> mismatching_class_distances;

            // Find all clusters within threshold of this GT point
            for (size_t i = 0; i < clusters.size(); ++i)
            {
                const auto &cluster = clusters[i];
                double dist_sq = sqdist(cluster.center, gt_point.position);

                if (dist_sq <= threshold_sq)
                {
                    double distance = std::sqrt(dist_sq);
                    matched_cluster_indices.insert(i);

                    if (cluster.class_id == gt_point.class_id)
                    {
                        matching_class_clusters.push_back(cluster);
                        matching_class_distances.push_back(distance);

                        if (verbose)
                        {
                            RCLCPP_DEBUG(node_->get_logger(),
                                         "✓ Cluster %zu (class %d, %zu voxels) matched GT marker %d at dist=%.3f m",
                                         i, cluster.class_id, cluster.points.size(),
                                         gt_point.id, distance);
                        }
                    }
                    else
                    {
                        mismatching_class_clusters.push_back(cluster);
                        mismatching_class_distances.push_back(distance);

                        if (verbose)
                        {
                            RCLCPP_WARN(node_->get_logger(),
                                        "✗ Cluster %zu (class %d) matched GT marker %d (class %d) but classes differ! dist=%.3f m",
                                        i, cluster.class_id, gt_point.id, gt_point.class_id, distance);
                        }
                    }
                }
            }

            // Create match entries for this GT point
            if (!matching_class_clusters.empty())
            {
                Match match;
                match.gt_point = gt_point;
                match.clusters = matching_class_clusters;
                match.distances = matching_class_distances;
                result.correct_matches.push_back(match);
            }

            if (!mismatching_class_clusters.empty())
            {
                Match match;
                match.gt_point = gt_point;
                match.clusters = mismatching_class_clusters;
                match.distances = mismatching_class_distances;
                result.class_mismatches.push_back(match);
            }

            // If no clusters matched this GT point, it's a false negative
            if (matching_class_clusters.empty() && mismatching_class_clusters.empty())
            {
                result.unmatched_gt.push_back(gt_point);
            }
        }

        // Collect unmatched clusters (false positives)
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            if (matched_cluster_indices.find(i) == matched_cluster_indices.end())
            {
                result.unmatched_clusters.push_back(clusters[i]);
            }
        }

        if (verbose)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Matching complete: %zu correct GT points, %zu mismatched GT points, %zu unmatched GT, %zu unmatched clusters",
                        result.correct_matches.size(), result.class_mismatches.size(),
                        result.unmatched_gt.size(), result.unmatched_clusters.size());
        }

        return result;
    }

    std::vector<ClassMetrics> OctoMapInterface::evaluateMatchResults(const MatchResult &result, bool verbose) const
    {
        // Debug: Print input summary
        RCLCPP_DEBUG(node_->get_logger(), "\n=== DEBUG: Evaluation Input ===");
        RCLCPP_DEBUG(node_->get_logger(), "GT classes count: %zu", gt_classes_.size());
        RCLCPP_DEBUG(node_->get_logger(), "Correct matches: %zu", result.correct_matches.size());
        RCLCPP_DEBUG(node_->get_logger(), "Class mismatches: %zu", result.class_mismatches.size());
        RCLCPP_DEBUG(node_->get_logger(), "Unmatched GT points: %zu", result.unmatched_gt.size());
        RCLCPP_DEBUG(node_->get_logger(), "Unmatched clusters: %zu", result.unmatched_clusters.size());

        // Determine the number of unique classes in GT and predictions and fill out the metrics vector
        std::vector<ClassMetrics> metrics;
        for (size_t i = 0; i < gt_classes_.size(); ++i)
        {
            // Create a ClassMetrics entry for each class and define the class_id
            ClassMetrics cm;
            cm.class_id = gt_classes_[i];

            RCLCPP_DEBUG(node_->get_logger(), "\n--- Processing class %d ---", cm.class_id);

            // Count true positive clusters for this class by summing clusters from correct matches that belong to this class
            for (const auto &match : result.correct_matches)
            {
                RCLCPP_DEBUG(node_->get_logger(), "Checking correct match: GT class=%d vs target class=%d", match.gt_point.class_id, cm.class_id);
                if (match.gt_point.class_id == cm.class_id)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Match! Adding %zu clusters", match.clusters.size());
                    cm.tp_clusters += static_cast<int>(match.clusters.size());
                }
            }

            // Count false positives for this class by summing unmatched clusters that belong to this class
            for (const auto &cluster : result.unmatched_clusters)
            {
                if (cluster.class_id == cm.class_id)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Unmatched cluster: class=%d", cluster.class_id);
                    cm.fp_clusters += 1;
                }
            }
            
            // The class mismatches also are counted as false positives for the predicted clusters
            for (const auto &mismatch : result.class_mismatches)
            {
                for (const auto &cluster : mismatch.clusters)
                {                   
                    if (cluster.class_id == cm.class_id)
                    {
                        RCLCPP_DEBUG(node_->get_logger(), "Mismatched cluster: cluster_class=%d", cluster.class_id);
                        cm.fp_clusters += 1;
                    }
                }
            }

            // Count true positives for GT points of this class
            for (const auto &match : result.correct_matches)
            {
                RCLCPP_DEBUG(node_->get_logger(), "Checking TP point: GT class=%d vs target class=%d", match.gt_point.class_id, cm.class_id);
                if (match.gt_point.class_id == cm.class_id)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "TP point match!");
                    cm.tp_points += 1;
                }
            }

            // Count false negatives for GT points of this class
            for (const auto &gt : result.unmatched_gt)
            {
                RCLCPP_DEBUG(node_->get_logger(), "Checking FN point: GT class=%d vs target class=%d", gt.class_id, cm.class_id);
                if (gt.class_id == cm.class_id)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "FN point match!");
                    cm.fn_points += 1;
                }
            }
            
            // The class mismatches also are counted as false negatives for the GT points
            for (const auto &mismatch : result.class_mismatches)
            {
                if (mismatch.gt_point.class_id == cm.class_id)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "FN point (mismatch): GT marker %d", mismatch.gt_point.id);
                    cm.fn_points += 1;
                }
            }
            
            RCLCPP_DEBUG(node_->get_logger(), "Final counts for class %d: TP_clusters=%d, FP_clusters=%d, TP_points=%d, FN_points=%d",
                        cm.class_id, cm.tp_clusters, cm.fp_clusters, cm.tp_points, cm.fn_points);
            
            // Compute the precision, recall, and F1 score for this class
            cm.precision = (cm.tp_clusters + cm.fp_clusters) > 0 ? static_cast<double>(cm.tp_clusters) / (cm.tp_clusters + cm.fp_clusters) : 0.0;
            cm.recall = (cm.tp_points + cm.fn_points) > 0 ? static_cast<double>(cm.tp_points) / (cm.tp_points + cm.fn_points) : 0.0;
            cm.f1_score = (cm.precision + cm.recall) > 0 ? 2.0 * cm.precision * cm.recall / (cm.precision + cm.recall) : 0.0;
            
            // Push the completed metrics object to the vector AFTER all modifications
            metrics.push_back(cm);
        }

        // Verbose output
        if (verbose)
        {
            // List correctly matched GT points
            if (!result.correct_matches.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "\nCorrectly Matched GT Points:");
                for (const auto &match : result.correct_matches)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "  Marker %d (class %d) at (%.2f, %.2f, %.2f) - %zu clusters:",
                                match.gt_point.id, match.gt_point.class_id,
                                match.gt_point.position.x(), match.gt_point.position.y(),
                                match.gt_point.position.z(), match.clusters.size());
                    for (size_t i = 0; i < match.clusters.size(); ++i)
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "    - Cluster (class %d, %zu voxels) at dist=%.3f m",
                                    match.clusters[i].class_id, match.clusters[i].points.size(),
                                    match.distances[i]);
                    }
                }
            }

            // List class mismatches
            if (!result.class_mismatches.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "\nGT Points with Class Mismatches:");
                for (const auto &match : result.class_mismatches)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "  Marker %d (class %d) at (%.2f, %.2f, %.2f) - %zu mismatched clusters:",
                                match.gt_point.id, match.gt_point.class_id,
                                match.gt_point.position.x(), match.gt_point.position.y(),
                                match.gt_point.position.z(), match.clusters.size());
                    for (size_t i = 0; i < match.clusters.size(); ++i)
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "    - Cluster (class %d, %zu voxels) at dist=%.3f m",
                                    match.clusters[i].class_id, match.clusters[i].points.size(),
                                    match.distances[i]);
                    }
                }
            }

            // List unmatched GT points
            if (!result.unmatched_gt.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "\nUnmatched GT Points:");
                for (const auto &gt : result.unmatched_gt)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "  Marker %d (class %d) at (%.2f, %.2f, %.2f)",
                                gt.id, gt.class_id,
                                gt.position.x(), gt.position.y(), gt.position.z());
                }
            }

            // List unmatched clusters
            if (!result.unmatched_clusters.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "\nUnmatched Clusters:");
                for (const auto &cluster : result.unmatched_clusters)
                {                    RCLCPP_INFO(node_->get_logger(),
                                "  Cluster (class %d, %zu voxels) at (%.2f, %.2f, %.2f)",
                                cluster.class_id, cluster.points.size(),
                                cluster.center.x(), cluster.center.y(), cluster.center.z());
                }
            }
        }
        
        return metrics;
    }

    std::pair<size_t, size_t> OctoMapInterface::getVoxelCounts() const
    {
        std::shared_lock lk(mtx_);

        bool has_tree = std::visit([](auto&& tree) { return tree != nullptr; }, tree_);
        if (!has_tree)
        {
            return {0, 0};
        }

        size_t occupied = 0, free_space = 0;
        std::visit([&](auto&& tree) {
            for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
            {
                if (tree->isNodeOccupied(*it))
                    occupied++;
                else
                    free_space++;
            }
        }, tree_);

        return {occupied, free_space};
    }

    double OctoMapInterface::calculateCoverage() const
    {
        std::shared_lock lk(mtx_);
        
        // Need valid bounding box to calculate coverage
        if (!has_valid_bbox_)
            return 0.0;
        
        // Get mapped voxel counts
        lk.unlock();  // Unlock before calling getVoxelCounts which will lock again
        auto [occupied, free_space] = getVoxelCounts();
        lk.lock();  // Re-lock to access resolution safely
        
        size_t total_mapped = occupied + free_space;
        if (total_mapped == 0)
            return 0.0;
        
        // Calculate bounding box volume
        double bbox_volume = (bbox_max_.x() - bbox_min_.x()) * 
                            (bbox_max_.y() - bbox_min_.y()) * 
                            (bbox_max_.z() - bbox_min_.z());
        
        if (bbox_volume <= 0.0)
            return 0.0;
        
        // Calculate volume of a single voxel
        double voxel_volume = resolution_ * resolution_ * resolution_;
        
        // Calculate theoretical maximum number of voxels that could fit in bbox
        double theoretical_max_voxels = bbox_volume / voxel_volume;
        
        // Calculate coverage percentage
        return (static_cast<double>(total_mapped) / theoretical_max_voxels) * 100.0;
    }

    void OctoMapInterface::printOctomapStats() const
    {
        std::shared_lock lk(mtx_);

        bool has_tree = std::visit([](auto&& tree) { return tree != nullptr; }, tree_);
        if (!has_tree)
        {
            RCLCPP_WARN(node_->get_logger(), "No octomap available");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "\n=== Octomap Statistics ===");

        // Common stats for both tree types
        std::visit([&](auto&& tree) {
            RCLCPP_INFO(node_->get_logger(), "Resolution: %.4f m", tree->getResolution());
            RCLCPP_INFO(node_->get_logger(), "Total nodes: %zu", tree->size());
            RCLCPP_INFO(node_->get_logger(), "Leaf nodes: %zu", tree->getNumLeafNodes());
        }, tree_);

        // Get voxel counts and coverage
        auto [occupied, free_space] = getVoxelCounts();
        double coverage = calculateCoverage();
        
        RCLCPP_INFO(node_->get_logger(), "Occupied voxels: %zu", occupied);
        RCLCPP_INFO(node_->get_logger(), "Free voxels: %zu", free_space);
        RCLCPP_INFO(node_->get_logger(), "Coverage: %.2f%%", coverage);

        // Display semantic class information if this is a semantic tree
        if (tree_type_ == OctoMapType::SEMANTIC)
        {
            auto class_counts = getSemanticClassCounts();
            if (!class_counts.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "Semantic Classes:");
                for (const auto &[class_id, count] : class_counts)
                {
                    RCLCPP_INFO(node_->get_logger(), "Class %d: %zu occupied voxels", class_id, count);
                }
            }
        }
        RCLCPP_INFO(node_->get_logger(), "==========================\n");
    }

    std::vector<ClassMetrics> OctoMapInterface::evaluateSemanticOctomap(double threshold_radius, bool verbose)
    {
        if (!isSemanticTree())
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot evaluate: current tree is not semantic");
            return {};
        }

        if (gt_points_.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot evaluate: no ground truth points loaded");
            return {};
        }

        if (verbose)
        {
            RCLCPP_INFO(node_->get_logger(), "\n=== Semantic Octomap Evaluation ===");
            RCLCPP_INFO(node_->get_logger(), "Ground truth points: %zu", gt_points_.size());
            RCLCPP_INFO(node_->get_logger(), "Match threshold radius: %.3f m", threshold_radius);
        }

        // Step 1: Cluster semantic voxels by class
        auto clusters = clusterSemanticVoxels(verbose);

        // Step 2: Match clusters to ground truth
        auto match_result = matchClustersToGroundTruth(clusters, threshold_radius, verbose);

        // Step 3: Evaluate and print metrics
        return evaluateMatchResults(match_result, verbose);
    }

}
