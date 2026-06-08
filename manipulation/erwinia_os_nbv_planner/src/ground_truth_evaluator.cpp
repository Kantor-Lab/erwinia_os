#include "erwinia_os_nbv_planner/ground_truth_evaluator.hpp"

#include "erwinia_os_nbv_planner/octomap_interface.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace erwinia_os_nbv_planner
{
namespace
{
double sqdist(const octomap::point3d &a, const octomap::point3d &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    const double dz = a.z() - b.z();
    return dx * dx + dy * dy + dz * dz;
}
} // namespace

GroundTruthEvaluator::GroundTruthEvaluator(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool GroundTruthEvaluator::loadGroundTruthFile(const std::string &file_path)
{
    try
    {
        YAML::Node root = YAML::LoadFile(file_path);
        if (!root["version"] || root["version"].as<int>() != 1)
        {
            RCLCPP_ERROR(node_->get_logger(), "Unsupported GT format in '%s'", file_path.c_str());
            return false;
        }

        if (!root["segments"] || !root["segments"].IsSequence())
        {
            RCLCPP_ERROR(node_->get_logger(), "GT file '%s' is missing a 'segments' array", file_path.c_str());
            return false;
        }

        gt_segments_.clear();
        gt_file_path_ = file_path;
        gt_frame_id_.clear();
        bool normalized_positive_class_ids = false;
        if (root["frame_id"])
        {
            gt_frame_id_ = root["frame_id"].as<std::string>();
        }

        for (const auto &segment : root["segments"])
        {
            if (!segment["classId"] || !segment["clusterIndex"] || !segment["centroid"])
            {
                RCLCPP_ERROR(node_->get_logger(), "GT segment is missing required fields in '%s'", file_path.c_str());
                return false;
            }

            const YAML::Node centroid = segment["centroid"];
            if (!centroid.IsSequence() || centroid.size() != 3)
            {
                RCLCPP_ERROR(node_->get_logger(), "GT segment centroid must contain exactly 3 values in '%s'", file_path.c_str());
                return false;
            }

            GroundTruthSegment gt_segment;
            const int32_t raw_class_id = segment["classId"].as<int32_t>();
            gt_segment.class_id = raw_class_id > 0 ? raw_class_id - 1 : raw_class_id;
            normalized_positive_class_ids = normalized_positive_class_ids || (raw_class_id > 0);
            gt_segment.class_name = segment["className"] ? segment["className"].as<std::string>() : "";
            gt_segment.cluster_index = segment["clusterIndex"].as<int32_t>();
            gt_segment.count = segment["count"] ? segment["count"].as<int32_t>() : 0;
            gt_segment.position = octomap::point3d(
                centroid[0].as<double>(),
                centroid[1].as<double>(),
                centroid[2].as<double>());
            gt_segment.id = std::to_string(gt_segment.class_id) + ":" + std::to_string(gt_segment.cluster_index);

            gt_segments_.push_back(gt_segment);
        }

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu GT segments from %s",
                    gt_segments_.size(), gt_file_path_.c_str());
        if (normalized_positive_class_ids)
        {
            RCLCPP_INFO(node_->get_logger(), "Normalized positive GT class IDs from 1-based JSON indexing to 0-based internal indexing");
        }
        return true;
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed parsing GT file '%s': %s", file_path.c_str(), e.what());
        return false;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed loading GT file '%s': %s", file_path.c_str(), e.what());
        return false;
    }
}

void GroundTruthEvaluator::setPointTransform(
    const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation)
{
    gt_translation_ = translation;
    gt_rotation_ = rotation.normalized();
}

std::vector<GroundTruthSegment> GroundTruthEvaluator::getGroundTruthInFrame(const std::string &target_frame) const
{
    std::vector<GroundTruthSegment> transformed = gt_segments_;

    const bool has_user_transform =
        !gt_translation_.isZero() ||
        gt_rotation_.angularDistance(Eigen::Quaterniond::Identity()) > 1e-9;

    if (has_user_transform)
    {
        for (auto &seg : transformed)
        {
            Eigen::Vector3d p(seg.position.x(), seg.position.y(), seg.position.z());
            p = gt_rotation_ * p + gt_translation_;
            seg.position = octomap::point3d(p.x(), p.y(), p.z());
        }
    }

    if (gt_frame_id_.empty() || target_frame.empty() || gt_frame_id_ == target_frame)
    {
        return transformed;
    }

    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        target_frame, gt_frame_id_, tf2::TimePointZero);

    for (auto &gt_segment : transformed)
    {
        geometry_msgs::msg::PointStamped in_point;
        geometry_msgs::msg::PointStamped out_point;
        in_point.header.frame_id = gt_frame_id_;
        in_point.point.x = gt_segment.position.x();
        in_point.point.y = gt_segment.position.y();
        in_point.point.z = gt_segment.position.z();
        tf2::doTransform(in_point, out_point, transform);
        gt_segment.position = octomap::point3d(
            out_point.point.x,
            out_point.point.y,
            out_point.point.z);
    }

    return transformed;
}

PredictedClusterSummary GroundTruthEvaluator::summarizeCluster(const Cluster &cluster)
{
    PredictedClusterSummary summary;
    summary.label = cluster.label;
    summary.class_id = cluster.class_id;
    summary.center = cluster.center;
    summary.size = cluster.size;
    summary.max_confidence = cluster.max_confidence;
    summary.voxels = cluster.points;
    return summary;
}

ViewpointEvaluation GroundTruthEvaluator::buildViewpointEvaluation(
    int viewpoint_index,
    double time_seconds,
    const std::optional<geometry_msgs::msg::Pose> &camera_pose,
    const std::vector<Cluster> &clusters,
    const std::string &octomap_frame_id,
    const std::optional<std::pair<octomap::point3d, octomap::point3d>> &bbox,
    int occupied_voxels,
    int free_voxels,
    double bbox_coverage) const
{
    ViewpointEvaluation evaluation;
    evaluation.viewpoint_index = viewpoint_index;
    evaluation.time = time_seconds;
    evaluation.occupied_voxels = occupied_voxels;
    evaluation.free_voxels = free_voxels;
    evaluation.bbox_coverage = bbox_coverage;
    evaluation.gt_segments = getGroundTruthInFrame(octomap_frame_id);
    if (bbox.has_value())
    {
        const auto &[bbx_min, bbx_max] = bbox.value();
        auto &segs = evaluation.gt_segments;
        segs.erase(
            std::remove_if(segs.begin(), segs.end(),
                [&](const GroundTruthSegment &gt_segment)
                {
                    const auto &p = gt_segment.position;
                    if (p.x() < bbx_min.x() || p.x() > bbx_max.x() ||
                        p.y() < bbx_min.y() || p.y() > bbx_max.y() ||
                        p.z() < bbx_min.z() || p.z() > bbx_max.z())
                    {
                        RCLCPP_WARN(
                            node_->get_logger(),
                            "GT segment %s (count=%d) at (%.3f, %.3f, %.3f) is outside "
                            "the octomap bounding box [(%.3f,%.3f,%.3f)-(%.3f,%.3f,%.3f)] — skipping",
                            gt_segment.id.c_str(), gt_segment.count,
                            p.x(), p.y(), p.z(),
                            bbx_min.x(), bbx_min.y(), bbx_min.z(),
                            bbx_max.x(), bbx_max.y(), bbx_max.z());
                        return true;
                    }
                    return false;
                }),
            segs.end());
    }
    if (camera_pose.has_value())
    {
        evaluation.camera_pose.valid = true;
        evaluation.camera_pose.pose = camera_pose.value();
    }

    for (const auto &cluster : clusters)
    {
        evaluation.predicted_clusters.push_back(summarizeCluster(cluster));
    }

    for (const auto &gt_segment : evaluation.gt_segments)
    {
        for (const auto &cluster : clusters)
        {
            PairwiseDistance pairwise;
            pairwise.gt_id = gt_segment.id;
            pairwise.predicted_label = cluster.label;
            pairwise.gt_class_id = gt_segment.class_id;
            pairwise.predicted_class_id = cluster.class_id;
            pairwise.same_class = cluster.class_id == gt_segment.class_id;
            if (cluster.points.empty())
            {
                pairwise.distance_m = std::sqrt(sqdist(cluster.center, gt_segment.position));
            }
            else
            {
                double min_dist_sq = std::numeric_limits<double>::max();
                for (const auto &point : cluster.points)
                    min_dist_sq = std::min(min_dist_sq, sqdist(point, gt_segment.position));
                pairwise.distance_m = std::sqrt(min_dist_sq);
            }
            evaluation.pairwise_distances.push_back(pairwise);
        }
    }
    return evaluation;
}

std::string GroundTruthEvaluator::escapeJson(const std::string &value)
{
    std::ostringstream oss;
    for (const char ch : value)
    {
        switch (ch)
        {
        case '\\':
            oss << "\\\\";
            break;
        case '"':
            oss << "\\\"";
            break;
        case '\n':
            oss << "\\n";
            break;
        case '\r':
            oss << "\\r";
            break;
        case '\t':
            oss << "\\t";
            break;
        default:
            oss << ch;
            break;
        }
    }
    return oss.str();
}

bool GroundTruthEvaluator::writeEvaluationsToJson(
    const std::vector<ViewpointEvaluation> &evaluations,
    const std::string &output_path) const
{
    std::ofstream out(output_path);
    if (!out.is_open())
    {
        return false;
    }

    out << "{\n";
    out << "  \"groundTruthFile\": \"" << escapeJson(gt_file_path_) << "\",\n";
    out << "  \"groundTruthFrameId\": \"" << escapeJson(gt_frame_id_) << "\",\n";
    out << "  \"groundTruthTranslation\": ["
        << gt_translation_.x() << "," << gt_translation_.y() << "," << gt_translation_.z() << "],\n";
    out << "  \"groundTruthRotation\": ["
        << gt_rotation_.x() << "," << gt_rotation_.y() << ","
        << gt_rotation_.z() << "," << gt_rotation_.w() << "],\n";
    out << "  \"viewpoints\": [\n";
    for (size_t i = 0; i < evaluations.size(); ++i)
    {
        const auto &evaluation = evaluations[i];
        out << "    {\n";
        out << "      \"viewpointIndex\": " << evaluation.viewpoint_index << ",\n";
        out << "      \"timeSec\": " << std::fixed << std::setprecision(6) << evaluation.time << ",\n";
        out << "      \"cameraPose\": ";
        if (evaluation.camera_pose.valid)
        {
            const auto &pose = evaluation.camera_pose.pose;
            out << "{\"position\":[" << pose.position.x << "," << pose.position.y << "," << pose.position.z
                << "],\"orientation\":[" << pose.orientation.x << "," << pose.orientation.y << ","
                << pose.orientation.z << "," << pose.orientation.w << "]}";
        }
        else
        {
            out << "null";
        }
        out << ",\n";
        out << "      \"occupiedVoxels\": " << evaluation.occupied_voxels << ",\n";
        out << "      \"freeVoxels\": " << evaluation.free_voxels << ",\n";
        out << "      \"bboxCoverage\": " << std::fixed << std::setprecision(6) << evaluation.bbox_coverage << ",\n";

        out << "      \"groundTruthSegments\": [\n";
        for (size_t j = 0; j < evaluation.gt_segments.size(); ++j)
        {
            const auto &gt = evaluation.gt_segments[j];
            out << "        {\"id\":\"" << escapeJson(gt.id) << "\",\"classId\":" << gt.class_id
                << ",\"className\":\"" << escapeJson(gt.class_name) << "\",\"clusterIndex\":" << gt.cluster_index
                << ",\"count\":" << gt.count << ",\"centroid\":[" << gt.position.x() << ","
                << gt.position.y() << "," << gt.position.z() << "]}";
            out << (j + 1 < evaluation.gt_segments.size() ? ",\n" : "\n");
        }
        out << "      ],\n";

        out << "      \"predictedClusters\": [\n";
        for (size_t j = 0; j < evaluation.predicted_clusters.size(); ++j)
        {
            const auto &cluster = evaluation.predicted_clusters[j];
            out << "        {\"label\":" << cluster.label << ",\"classId\":" << cluster.class_id
                << ",\"size\":" << cluster.size << ",\"maxConfidence\":" << cluster.max_confidence
                << ",\"center\":[" << cluster.center.x() << ","
                << cluster.center.y() << "," << cluster.center.z() << "]";
            if (!cluster.voxels.empty())
            {
                out << ",\"voxels\":[";
                for (size_t k = 0; k < cluster.voxels.size(); ++k)
                {
                    out << "[" << cluster.voxels[k].x() << "," << cluster.voxels[k].y() << "," << cluster.voxels[k].z() << "]";
                    out << (k + 1 < cluster.voxels.size() ? "," : "");
                }
                out << "]";
            }
            out << "}";
            out << (j + 1 < evaluation.predicted_clusters.size() ? ",\n" : "\n");
        }
        out << "      ],\n";

        out << "      \"pairwiseDistances\": [\n";
        for (size_t j = 0; j < evaluation.pairwise_distances.size(); ++j)
        {
            const auto &distance = evaluation.pairwise_distances[j];
            out << "        {\"gtId\":\"" << escapeJson(distance.gt_id) << "\",\"predictedLabel\":"
                << distance.predicted_label << ",\"gtClassId\":" << distance.gt_class_id
                << ",\"predictedClassId\":" << distance.predicted_class_id << ",\"distanceM\":"
                << distance.distance_m << ",\"sameClass\":" << (distance.same_class ? "true" : "false") << "}";
            out << (j + 1 < evaluation.pairwise_distances.size() ? ",\n" : "\n");
        }
        out << "      ],\n";
        out << "      \"gtClassIndexing\": \"zero_based\",\n";
        out << "      \"predictedClassIndexing\": \"zero_based\"\n";
        out << "    }" << (i + 1 < evaluations.size() ? ",\n" : "\n");
    }
    out << "  ]\n";
    out << "}\n";

    return true;
}

std::array<uint8_t, 3> GroundTruthEvaluator::classColor(int32_t class_id)
{
    if (class_id < 0)
    {
        return {128, 128, 128};
    }

    static constexpr std::array<std::array<uint8_t, 3>, 20> palette{{
        {{0, 150, 255}},
        {{230, 25, 75}},
        {{60, 180, 75}},
        {{255, 225, 25}},
        {{245, 130, 48}},
        {{145, 30, 180}},
        {{70, 240, 240}},
        {{240, 50, 230}},
        {{210, 245, 60}},
        {{250, 190, 212}},
        {{0, 128, 128}},
        {{220, 190, 255}},
        {{170, 110, 40}},
        {{255, 250, 200}},
        {{128, 0, 0}},
        {{170, 255, 195}},
        {{128, 128, 0}},
        {{255, 215, 180}},
        {{0, 0, 128}},
        {{128, 128, 128}},
    }};

    if (class_id < static_cast<int32_t>(palette.size()))
    {
        return palette[static_cast<size_t>(class_id)];
    }

    return {
        static_cast<uint8_t>((class_id * 53) % 255),
        static_cast<uint8_t>((class_id * 97) % 255),
        static_cast<uint8_t>((class_id * 193) % 255)};
}

bool GroundTruthEvaluator::exportVoxelSnapshot(
    const OctoMapInterface &octomap_interface,
    int viewpoint_index,
    const std::string &output_dir) const
{
    namespace fs = std::filesystem;
    fs::create_directories(output_dir);

    const auto voxels = octomap_interface.getVoxelSamples(false);
    if (voxels.empty())
    {
        return false;
    }

    std::ostringstream path_builder;
    path_builder << output_dir << "/viewpoint_" << std::setw(4) << std::setfill('0') << viewpoint_index << ".ply";
    const std::string output_path = path_builder.str();

    std::ofstream out(output_path);
    if (!out.is_open())
    {
        return false;
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << voxels.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";

    for (const auto &voxel : voxels)
    {
        const auto color = classColor(voxel.class_id);
        out << voxel.center.x() << " " << voxel.center.y() << " " << voxel.center.z() << " "
            << static_cast<int>(color[0]) << " "
            << static_cast<int>(color[1]) << " "
            << static_cast<int>(color[2]) << "\n";
    }

    return true;
}

void GroundTruthEvaluator::logViewpointEvaluation(const ViewpointEvaluation &evaluation) const
{
    RCLCPP_INFO(node_->get_logger(), "\n=== Evaluation Results ===");
    RCLCPP_INFO(node_->get_logger(), "  Viewpoint: %d", evaluation.viewpoint_index);
    RCLCPP_INFO(node_->get_logger(), "  GT segments: %zu", evaluation.gt_segments.size());
    RCLCPP_INFO(node_->get_logger(), "  Predicted clusters: %zu", evaluation.predicted_clusters.size());
    RCLCPP_INFO(node_->get_logger(), "  Pairwise distances: %zu", evaluation.pairwise_distances.size());
    RCLCPP_INFO(node_->get_logger(), "  Occupied voxels: %d", evaluation.occupied_voxels);
    RCLCPP_INFO(node_->get_logger(), "  Free voxels: %d", evaluation.free_voxels);
    RCLCPP_INFO(node_->get_logger(), "  BBox coverage: %.4f", evaluation.bbox_coverage);
}

} // namespace erwinia_os_nbv_planner
