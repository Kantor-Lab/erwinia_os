#pragma once

#include <octomap/octomap.h>
#include <vector>

namespace erwinia_os_nbv_planner
{
    struct Cluster
    {
        int label;
        int32_t class_id = -1;
        octomap::point3d center;
        int size;
        std::vector<octomap::point3d> points;
    };

    struct SemanticPoint
    {
        int id;                 // Unique identifier for the semantic point (e.g., marker ID)
        int32_t class_id = -1;
        octomap::point3d position;  // 3D position in map frame
    };

    /**
     * @brief Result of matching semantic clusters to ground truth points
     */
    struct Match
    {
        SemanticPoint gt_point;
        std::vector<Cluster> clusters;
        std::vector<double> distances;
    };
    
    struct MatchResult
    {
        std::vector<Match> correct_matches;      // GT and clusters with matching class
        std::vector<Match> class_mismatches;     // GT and clusters matched but wrong class
        std::vector<SemanticPoint> unmatched_gt;
        std::vector<Cluster> unmatched_clusters;
    };

    /**
     * @brief Evaluation metrics for a specific semantic class
     */
    struct ClassMetrics
    {
        int32_t class_id;

        int32_t tp_clusters = 0;  // Class id clusters correctly matched to GT points of this class
        int32_t fp_clusters = 0;  // Class id clusters incorrectly matched (class mismatch or no match)

        int32_t tp_points = 0;    // GT points correctly matched to clusters of the same class
        int32_t fn_points = 0;    // GT points not matched to any cluster (class mismatch or no match)
        
        double precision; // Computed using the cluster counts: tp_clusters / (tp_clusters + fp_clusters)
        double recall;    // Computed using the GT point counts: tp_points / (tp_points + fn_points)
        double f1_score;  // Harmonic mean of precision and recall: 2 * (precision * recall) / (precision + recall)
    };

    /**
     * @brief Resulting evaluation metrics for all semantic classes at a single point in time
     * 
     * Note: To track metrics over time, use std::vector<EvaluationMetrics>
     */
    struct EvaluationMetrics
    {
        double time;                              // Timestamp (seconds)
        std::vector<ClassMetrics> class_metrics;  // Metrics for each semantic class
        int free_voxels;                          // Count of free voxels
        int occupied_voxels;                      // Count of occupied voxels
        double bbox_coverage;                       // Percentage of bounding box covered by occupied voxels
    };
}