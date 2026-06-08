#pragma once

#include "erwinia_os_nbv_planner/nbv_types.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace erwinia_os_nbv_planner
{
    class OctoMapInterface;

    class GroundTruthEvaluator
    {
    public:
        explicit GroundTruthEvaluator(const rclcpp::Node::SharedPtr &node);

        bool loadGroundTruthFile(const std::string &file_path);
        bool hasGroundTruth() const { return !gt_segments_.empty(); }

        void setPointTransform(const Eigen::Vector3d &translation,
                               const Eigen::Quaterniond &rotation);

        const std::vector<GroundTruthSegment> &getGroundTruthSegments() const { return gt_segments_; }
        const std::string &getGroundTruthFrameId() const { return gt_frame_id_; }
        const std::string &getGroundTruthFilePath() const { return gt_file_path_; }
        std::vector<GroundTruthSegment> getGroundTruthInFrame(
            const std::string &target_frame) const;

        ViewpointEvaluation buildViewpointEvaluation(
            int viewpoint_index,
            double time_seconds,
            const std::optional<geometry_msgs::msg::Pose> &camera_pose,
            const std::vector<Cluster> &clusters,
            const std::string &octomap_frame_id,
            const std::optional<std::pair<octomap::point3d, octomap::point3d>> &bbox,
            int occupied_voxels,
            int free_voxels,
            double bbox_coverage) const;

        bool writeEvaluationsToJson(
            const std::vector<ViewpointEvaluation> &evaluations,
            const std::string &output_path) const;

        bool exportVoxelSnapshot(
            const OctoMapInterface &octomap_interface,
            int viewpoint_index,
            const std::string &output_dir) const;

        void logViewpointEvaluation(const ViewpointEvaluation &evaluation) const;

    private:
        static PredictedClusterSummary summarizeCluster(const Cluster &cluster);
        static std::string escapeJson(const std::string &value);
        static std::array<uint8_t, 3> classColor(int32_t class_id);

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string gt_file_path_;
        std::string gt_frame_id_;
        std::vector<GroundTruthSegment> gt_segments_;

        Eigen::Vector3d gt_translation_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond gt_rotation_ = Eigen::Quaterniond::Identity();
    };
} // namespace erwinia_os_nbv_planner
