#pragma once

#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <cmath>

namespace erwinia_os_nbv_planner
{
namespace geometry_utils
{

/**
 * @brief Convert quaternion [x,y,z,w] to 3x3 rotation matrix
 * 
 * @param q Quaternion as array [x, y, z, w]
 * @return Eigen::Matrix3d Rotation matrix (world <- body)
 */
inline Eigen::Matrix3d quatToRotMat(const std::array<double, 4>& q)
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double xx = x*x, yy = y*y, zz = z*z;
    const double xy = x*y, xz = x*z, yz = y*z;
    const double wx = w*x, wy = w*y, wz = w*z;
    
    Eigen::Matrix3d R;
    R << 1 - 2*(yy + zz),   2*(xy - wz),       2*(xz + wy),
         2*(xy + wz),       1 - 2*(xx + zz),   2*(yz - wx),
         2*(xz - wy),       2*(yz + wx),       1 - 2*(xx + yy);
    
    return R;
}

/**
 * @brief Convert Eigen quaternion to array [x,y,z,w]
 * 
 * @param q Eigen quaternion
 * @return std::array<double, 4> Quaternion as [x, y, z, w]
 */
inline std::array<double, 4> eigenQuatToArray(const Eigen::Quaterniond& q)
{
    return {q.x(), q.y(), q.z(), q.w()};
}

/**
 * @brief Convert array [x,y,z,w] to Eigen quaternion
 * 
 * @param q Quaternion as array [x, y, z, w]
 * @return Eigen::Quaterniond Eigen quaternion
 */
inline Eigen::Quaterniond arrayToEigenQuat(const std::array<double, 4>& q)
{
    return Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
}

/**
 * @brief Compute minimal-rotation quaternion mapping unit vector a to b
 * 
 * @param a First unit vector
 * @param b Second unit vector
 * @return std::array<double, 4> Quaternion [x,y,z,w]
 */
inline std::array<double, 4> quatFromTwoVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    const Eigen::Vector3d a_norm = a.normalized();
    const Eigen::Vector3d b_norm = b.normalized();
    
    const double dot = std::clamp(a_norm.dot(b_norm), -1.0, 1.0);
    
    // Nearly parallel
    if (dot > 1.0 - 1e-8) {
        return {0.0, 0.0, 0.0, 1.0};  // Identity
    }
    
    // Nearly antiparallel (180°)
    if (dot < -1.0 + 1e-8) {
        // Pick any axis perpendicular to a
        Eigen::Vector3d axis = (std::abs(a_norm.x()) < 0.9) ? 
            Eigen::Vector3d(1.0, 0.0, 0.0) : Eigen::Vector3d(0.0, 1.0, 0.0);
        axis = axis - a_norm * a_norm.dot(axis);
        axis.normalize();
        return {axis.x(), axis.y(), axis.z(), 0.0};  // 180° rotation
    }
    
    // General case
    const Eigen::Vector3d axis = a_norm.cross(b_norm);
    const double s = std::sqrt((1.0 + dot) * 2.0);
    const double invs = 1.0 / s;
    
    return {axis.x() * invs, axis.y() * invs, axis.z() * invs, 0.5 * s};
}

/**
 * @brief Compute ray strides for image sampling
 * 
 * @param width Image width
 * @param height Image height
 * @param num_rays Desired number of rays (-1 for all pixels)
 * @return std::pair<int, int> (stride_u, stride_v)
 */
inline std::pair<int, int> computeRayStrides(int width, int height, int num_rays)
{
    if (num_rays <= 0) {
        return {1, 1};
    }
    
    const int total = width * height;
    const int step = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(total) / num_rays)));
    return {step, step};
}

/**
 * @brief Convert angle from degrees to radians
 * 
 * @param degrees Angle in degrees
 * @return double Angle in radians
 */
inline double deg2Rad(double degrees)
{
    return degrees * M_PI / 180.0;
}

/**
 * @brief Convert angle from radians to degrees
 * 
 * @param radians Angle in radians
 * @return double Angle in degrees
 */
inline double rad2Deg(double radians)
{
    return radians * 180.0 / M_PI;
}

/**
 * @brief Convert vector of angles from degrees to radians
 * 
 * @param degrees Vector of angles in degrees
 * @return std::vector<double> Vector of angles in radians
 */
inline std::vector<double> deg2Rad(const std::vector<double>& degrees)
{
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (const auto& deg : degrees)
    {
        radians.push_back(deg2Rad(deg));
    }
    return radians;
}

/**
 * @brief Convert vector of angles from radians to degrees
 * 
 * @param radians Vector of angles in radians
 * @return std::vector<double> Vector of angles in degrees
 */
inline std::vector<double> rad2Deg(const std::vector<double>& radians)
{
    std::vector<double> degrees;
    degrees.reserve(radians.size());
    for (const auto& rad : radians)
    {
        degrees.push_back(rad2Deg(rad));
    }
    return degrees;
}

/**
 * @brief Batch transform a vector of Eigen::Vector3d points using an Isometry3d transform.
 * @param points Input points to transform
 * @param transform Transformation to apply
 * @return Transformed points
 */
std::vector<Eigen::Vector3d> transformPointsBatch(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Isometry3d& transform)
{
    if (points.empty()) {
        return {};
    }
    
    // Pre-extract rotation and translation for efficiency
    const Eigen::Matrix3d& rotation = transform.rotation();
    const Eigen::Vector3d& translation = transform.translation();
    
    std::vector<Eigen::Vector3d> transformed_points;
    transformed_points.reserve(points.size());
    
    for (const auto& pt : points) {
        transformed_points.push_back(rotation * pt + translation);
    }
    
    return transformed_points;
}

/**
 * @brief Check if a transformation is (approximately) an identity transform
 * 
 * @param transform The Eigen::Isometry3d transform to check
 * @param epsilon Tolerance for near-zero comparisons (default 1e-6)
 * @return true if the transform is approximately identity, false otherwise
 */
inline bool isIdentityTransform(const Eigen::Isometry3d& transform, double epsilon = 1e-6)
{
    // Check if rotation is identity
    if (!transform.rotation().isIdentity(epsilon)) {
        return false;
    }
    
    // Check if translation is zero
    if (transform.translation().norm() > epsilon) {
        return false;
    }
    
    return true;
}

} // namespace geometry_utils
} // namespace erwinia_os_nbv_planner
