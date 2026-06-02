#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <octomap/octomap.h>

#include <string>
#include <vector>

namespace erwinia_os_nbv_planner
{
    struct Cluster
    {
        int label = -1;
        int32_t class_id = -1;
        octomap::point3d center;
        int size = 0;
        float max_confidence = 0.0f;
        std::vector<octomap::point3d> points;
    };

    struct GroundTruthSegment
    {
        std::string id;
        int32_t class_id = -1;
        std::string class_name;
        int32_t cluster_index = -1;
        int32_t count = 0;
        octomap::point3d position;
    };

    struct PredictedClusterSummary
    {
        int label = -1;
        int32_t class_id = -1;
        octomap::point3d center;
        int size = 0;
        float max_confidence = 0.0f;
    };

    struct PairwiseDistance
    {
        std::string gt_id;
        int predicted_label = -1;
        int32_t gt_class_id = -1;
        int32_t predicted_class_id = -1;
        double distance_m = 0.0;
        bool same_class = false;
    };

    struct CameraPoseRecord
    {
        bool valid = false;
        geometry_msgs::msg::Pose pose;
    };

    struct VoxelSample
    {
        octomap::point3d center;
        bool occupied = false;
        int32_t class_id = -1;
    };

    struct Match
    {
        GroundTruthSegment gt_point;
        std::vector<PredictedClusterSummary> clusters;
        std::vector<double> distances;
    };

    struct MatchResult
    {
        std::vector<Match> correct_matches;
        std::vector<Match> class_mismatches;
        std::vector<GroundTruthSegment> unmatched_gt;
        std::vector<PredictedClusterSummary> unmatched_clusters;
        std::vector<PairwiseDistance> pairwise_distances;
    };

    struct ClassMetrics
    {
        int32_t class_id = -1;
        int32_t tp_clusters = 0;
        int32_t fp_clusters = 0;
        int32_t tp_points = 0;
        int32_t fn_points = 0;
        double precision = 0.0;
        double recall = 0.0;
        double f1_score = 0.0;
    };

    struct ViewpointEvaluation
    {
        int viewpoint_index = -1;
        double time = 0.0;
        CameraPoseRecord camera_pose;
        std::vector<ClassMetrics> class_metrics;
        int free_voxels = 0;
        int occupied_voxels = 0;
        double bbox_coverage = 0.0;
        std::vector<GroundTruthSegment> gt_segments;
        std::vector<PredictedClusterSummary> predicted_clusters;
        std::vector<PairwiseDistance> pairwise_distances;
    };

    using EvaluationMetrics = ViewpointEvaluation;
} // namespace erwinia_os_nbv_planner
