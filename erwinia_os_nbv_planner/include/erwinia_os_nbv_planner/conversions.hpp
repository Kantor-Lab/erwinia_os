/**
 * @file conversions.hpp
 * @brief Utility functions for converting between different types (Eigen, geometry_msgs, octomap)
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <octomap/octomap_types.h>
#include <array>

namespace erwinia_os_nbv_planner
{
namespace conversions
{

// ============================================================================
// Eigen <-> geometry_msgs conversions
// ============================================================================

/**
 * @brief Convert Eigen::Vector3d to geometry_msgs::msg::Point
 */
inline geometry_msgs::msg::Point eigenToPoint(const Eigen::Vector3d& vec)
{
    geometry_msgs::msg::Point point;
    point.x = vec.x();
    point.y = vec.y();
    point.z = vec.z();
    return point;
}

/**
 * @brief Convert geometry_msgs::msg::Point to Eigen::Vector3d
 */
inline Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point& point)
{
    return Eigen::Vector3d(point.x, point.y, point.z);
}

/**
 * @brief Convert std::array<double, 4> [x,y,z,w] to geometry_msgs::msg::Quaternion
 */
inline geometry_msgs::msg::Quaternion arrayToQuaternion(const std::array<double, 4>& quat)
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = quat[0];
    quaternion.y = quat[1];
    quaternion.z = quat[2];
    quaternion.w = quat[3];
    return quaternion;
}

/**
 * @brief Convert geometry_msgs::msg::Quaternion to std::array<double, 4> [x,y,z,w]
 */
inline std::array<double, 4> quaternionToArray(const geometry_msgs::msg::Quaternion& quat)
{
    return {quat.x, quat.y, quat.z, quat.w};
}

/**
 * @brief Convert Eigen::Quaterniond to geometry_msgs::msg::Quaternion
 */
inline geometry_msgs::msg::Quaternion eigenQuatToMsg(const Eigen::Quaterniond& quat)
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = quat.x();
    quaternion.y = quat.y();
    quaternion.z = quat.z();
    quaternion.w = quat.w();
    return quaternion;
}

/**
 * @brief Convert geometry_msgs::msg::Quaternion to Eigen::Quaterniond
 */
inline Eigen::Quaterniond msgToEigenQuat(const geometry_msgs::msg::Quaternion& quat)
{
    return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

/**
 * @brief Convert Eigen::Quaterniond to std::array<double, 4> [x,y,z,w]
 */
inline std::array<double, 4> eigenQuatToArray(const Eigen::Quaterniond& quat)
{
    return {quat.x(), quat.y(), quat.z(), quat.w()};
}

/**
 * @brief Convert std::array<double, 4> [x,y,z,w] to Eigen::Quaterniond
 */
inline Eigen::Quaterniond arrayToEigenQuat(const std::array<double, 4>& quat)
{
    return Eigen::Quaterniond(quat[3], quat[0], quat[1], quat[2]);
}

/**
 * @brief Convert Eigen position and orientation to geometry_msgs::msg::Pose
 * @param position Eigen::Vector3d position
 * @param orientation std::array<double, 4> quaternion [x,y,z,w]
 */
inline geometry_msgs::msg::Pose eigenToPose(const Eigen::Vector3d& position, 
                                             const std::array<double, 4>& orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position = eigenToPoint(position);
    pose.orientation = arrayToQuaternion(orientation);
    return pose;
}

/**
 * @brief Convert Eigen position and quaternion to geometry_msgs::msg::Pose
 * @param position Eigen::Vector3d position
 * @param orientation Eigen::Quaterniond quaternion
 */
inline geometry_msgs::msg::Pose eigenToPose(const Eigen::Vector3d& position,
                                             const Eigen::Quaterniond& orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position = eigenToPoint(position);
    pose.orientation = eigenQuatToMsg(orientation);
    return pose;
}

/**
 * @brief Convert geometry_msgs::msg::Pose to Eigen position and orientation array
 * @param pose geometry_msgs::msg::Pose
 * @param position Output Eigen::Vector3d position
 * @param orientation Output std::array<double, 4> quaternion [x,y,z,w]
 */
inline void poseToEigen(const geometry_msgs::msg::Pose& pose,
                        Eigen::Vector3d& position,
                        std::array<double, 4>& orientation)
{
    position = pointToEigen(pose.position);
    orientation = quaternionToArray(pose.orientation);
}

/**
 * @brief Convert geometry_msgs::msg::Pose to Eigen position and quaternion
 * @param pose geometry_msgs::msg::Pose
 * @param position Output Eigen::Vector3d position
 * @param orientation Output Eigen::Quaterniond quaternion
 */
inline void poseToEigen(const geometry_msgs::msg::Pose& pose,
                        Eigen::Vector3d& position,
                        Eigen::Quaterniond& orientation)
{
    position = pointToEigen(pose.position);
    orientation = msgToEigenQuat(pose.orientation);
}

// ============================================================================
// Eigen <-> octomap conversions
// ============================================================================

/**
 * @brief Convert Eigen::Vector3d to octomap::point3d
 */
inline octomap::point3d eigenToOctomap(const Eigen::Vector3d& vec)
{
    return octomap::point3d(vec.x(), vec.y(), vec.z());
}

/**
 * @brief Convert octomap::point3d to Eigen::Vector3d
 */
inline Eigen::Vector3d octomapToEigen(const octomap::point3d& point)
{
    return Eigen::Vector3d(point.x(), point.y(), point.z());
}

// ============================================================================
// geometry_msgs <-> octomap conversions
// ============================================================================

/**
 * @brief Convert geometry_msgs::msg::Point to octomap::point3d
 */
inline octomap::point3d pointToOctomap(const geometry_msgs::msg::Point& point)
{
    return octomap::point3d(point.x, point.y, point.z);
}

/**
 * @brief Convert octomap::point3d to geometry_msgs::msg::Point
 */
inline geometry_msgs::msg::Point octomapToPoint(const octomap::point3d& point)
{
    geometry_msgs::msg::Point msg_point;
    msg_point.x = point.x();
    msg_point.y = point.y();
    msg_point.z = point.z();
    return msg_point;
}

/**
 * @brief Convert geometry_msgs::msg::Pose position to octomap::point3d
 */
inline octomap::point3d poseToOctomap(const geometry_msgs::msg::Pose& pose)
{
    return pointToOctomap(pose.position);
}

// ============================================================================
// Batch conversions
// ============================================================================

/**
 * @brief Convert vector of Eigen::Vector3d to vector of octomap::point3d
 */
inline std::vector<octomap::point3d> eigenVectorToOctomap(const std::vector<Eigen::Vector3d>& points)
{
    std::vector<octomap::point3d> octomap_points;
    octomap_points.reserve(points.size());
    for (const auto& point : points)
    {
        octomap_points.push_back(eigenToOctomap(point));
    }
    return octomap_points;
}

/**
 * @brief Convert vector of octomap::point3d to vector of Eigen::Vector3d
 */
inline std::vector<Eigen::Vector3d> octomapVectorToEigen(const std::vector<octomap::point3d>& points)
{
    std::vector<Eigen::Vector3d> eigen_points;
    eigen_points.reserve(points.size());
    for (const auto& point : points)
    {
        eigen_points.push_back(octomapToEigen(point));
    }
    return eigen_points;
}

/**
 * @brief Convert vector of geometry_msgs::msg::Point to vector of octomap::point3d
 */
inline std::vector<octomap::point3d> pointVectorToOctomap(const std::vector<geometry_msgs::msg::Point>& points)
{
    std::vector<octomap::point3d> octomap_points;
    octomap_points.reserve(points.size());
    for (const auto& point : points)
    {
        octomap_points.push_back(pointToOctomap(point));
    }
    return octomap_points;
}

/**
 * @brief Convert vector of octomap::point3d to vector of geometry_msgs::msg::Point
 */
inline std::vector<geometry_msgs::msg::Point> octomapVectorToPoint(const std::vector<octomap::point3d>& points)
{
    std::vector<geometry_msgs::msg::Point> msg_points;
    msg_points.reserve(points.size());
    for (const auto& point : points)
    {
        msg_points.push_back(octomapToPoint(point));
    }
    return msg_points;
}

} // namespace conversions
} // namespace erwinia_os_nbv_planner
