#pragma once

#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <optional>
#include <memory>
#include <octomap/octomap.h>

#include "erwinia_os_nbv_planner/viewpoint.hpp"
#include "erwinia_os_nbv_planner/geometry_utils.hpp"
#include "erwinia_os_nbv_planner/moveit_interface.hpp"
#include "erwinia_os_nbv_planner/octomap_interface.hpp"
#include "erwinia_os_nbv_planner/nbv_visualizer.hpp"

namespace erwinia_os_nbv_planner
{

/**
 * @brief Compute robot joint angles to achieve a given viewpoint
 * 
 * @param moveit_interface MoveIt interface for IK computation
 * @param viewpoint Viewpoint with position and orientation
 * @param camera_link Name of the camera link frame
 * @param ik_timeout IK timeout in seconds
 * @param ik_attempts Number of IK attempts
 * @return Joint angles if IK successful, nullopt otherwise
 */
std::optional<std::vector<double>> computeViewpointJointAngles(
    const std::shared_ptr<MoveItInterface>& moveit_interface,
    Viewpoint& viewpoint,
    const std::string& camera_link,
    double ik_timeout,
    int ik_attempts,
    const rclcpp::Logger& logger);

/**
 * @brief Generate viewpoints on a spherical cap
 * 
 * Creates evenly-spaced points and orientations over a spherical cap with
 * minimal roll (no twist). The cap is computed along the Z-axis of the
 * orientation frame.
 * 
 * @param position Cap center in world coordinates
 * @param orientation Base orientation quaternion [x,y,z,w] defining reference frame
 * @param radius Sphere radius
 * @param angular_resolution Desired angular spacing (radians)
 * @param max_theta Half-angle of the cap (radians)
 * @param look_at_center If true, orientations point inward; if false, outward
 * @param use_positive_z If true, cap along +Z axis; if false, along -Z axis
 * @return Vector of viewpoints on the spherical cap
 */
std::vector<Viewpoint> computeSphericalCap(
    const Eigen::Vector3d& position,
    const std::array<double, 4>& orientation,
    double radius,
    double angular_resolution,
    double max_theta,
    bool look_at_center = false,
    bool use_positive_z = true);

/**
 * @brief Generate viewpoints on a plane with spherical cap orientations
 * 
 * Creates a grid of positions on a plane and for each position generates
 * multiple orientations using a spherical cap distribution.
 * 
 * @param position Center point of the plane
 * @param orientation Quaternion [x,y,z,w] defining plane orientation
 * @param half_extent Half-size of the plane grid
 * @param spatial_resolution Distance between viewpoint positions
 * @param max_theta Maximum angle (radians) to vary roll/pitch
 * @param angular_resolution Angular step size (radians)
 * @return Vector of viewpoints with all position/orientation combinations
 */
std::vector<Viewpoint> generatePlanarSphericalCapCandidates(
    const Eigen::Vector3d& position,
    const std::array<double, 4>& orientation,
    double half_extent,
    double spatial_resolution,
    double max_theta,
    double angular_resolution);

/**
 * @brief Randomly sample viewpoints from a hemispherical shell
 * 
 * Uses Gaussian distribution biased toward the Z-axis with minimum distance
 * constraints between samples. The hemisphere is oriented along the Z-axis
 * of the base_orientation frame.
 * 
 * @param center Center point of the hemisphere
 * @param base_orientation Quaternion [x,y,z,w] defining hemisphere frame
 * @param min_radius Minimum radius of the hemispherical shell
 * @param max_radius Maximum radius (if equal to min_radius, samples on surface)
 * @param num_samples Number of viewpoints to sample
 * @param use_positive_z If true, hemisphere along +Z; if false, along -Z
 * @param z_bias_sigma Std deviation for Gaussian theta sampling (default 0.3)
 * @param min_distance Minimum distance between samples (default 0.1m)
 * @param max_attempts Maximum attempts before giving up (default 1000)
 * @return Vector of sampled viewpoints
 */
std::vector<Viewpoint> sampleViewsFromHemisphere(
    const Eigen::Vector3d& center,
    const std::array<double, 4>& base_orientation,
    double min_radius,
    double max_radius,
    int num_samples = 20,
    bool use_positive_z = false,
    double z_bias_sigma = 0.3,
    double min_distance = 0.1,
    int max_attempts = 1000,
    const rclcpp::Logger& logger = rclcpp::get_logger("viewpoint_generation"));

/**
 * @brief Wrapper around the sample views from hemisphere to generate frontier-based viewpoints
 */
std::vector<Viewpoint> generateFrontierBasedViewpoints(
    const std::vector<Eigen::Vector3d>& centers,
    const std::array<double, 4>& base_orientation,
    double min_radius,
    double max_radius,
    int num_samples = 20,
    bool use_positive_z = false,
    double z_bias_sigma = 0.3,
    double min_distance = 0.1,
    int max_attempts = 1000,
    const rclcpp::Logger& logger = rclcpp::get_logger("viewpoint_generation"));

/**
 * @brief Compute the plane intersection with the bounding box and return the corners and the distance to the midpoint.
 * @cam_position: Camera position to build the viewing direction from.
 * @return Pair of midplane corners and distance vector from the bbox midpoint to the camera position.
 */
std::pair<std::vector<Eigen::Vector3d>, Eigen::Vector3d> computePlane(
        const std::shared_ptr<OctoMapInterface> &octomap_interface,
        const Eigen::Vector3d &cam_position);

/**
 * @brief Generate viewpoints facing the plane defined by the corners and distance.
 * @corners: Four corners of the midplane.
 * @distance: Distance vector from the plane midpoint to the viewpoint positions.
 * @orientation_base: Base orientation for the viewpoints.
 * @overlap_ratio: Desired overlap ratio between viewpoint frustum cross-sections on the midplane.
 * @camera_hfov_rad: Camera horizontal field of view (radians).
 * @camera_vfov_rad: Camera vertical field of view (radians).
 * @return Pair of viewpoints and coverage planes.
 */
std::pair<std::vector<Viewpoint>, std::vector<std::vector<Eigen::Vector3d>>> generateViewpointsFromPlane(
    const std::vector<Eigen::Vector3d> &corners,
    const Eigen::Vector3d &distance,
    const Eigen::Quaterniond &orientation_base,
    double overlap_ratio,
    double camera_hfov_rad, 
    double camera_vfov_rad);

/**
 * @brief Generate spherical cap viewpoints for each viewpoint in the input grid.
 * @grid_viewpoints: Input grid of viewpoints to generate spherical caps from.
 * @orientation: Base orientation for the spherical cap.
 * @max_theta: Maximum angle for the spherical cap (radians).
 * @angular_resolution: Angular resolution for the spherical cap (radians).
 * @return Vector of viewpoints with spherical cap orientations.
 */
std::vector<Viewpoint> generateSphericalCaps(
    const std::vector<Viewpoint> &grid_viewpoints,
    const std::array<double, 4> &orientation,
    double max_theta,
    double angular_resolution);

/**
 * @brief Compute information gain for a viewpoint using geometric octomap
 * 
 * Information gain is the average number of unknown voxels discovered per ray.
 * Rays stop at max_range or when hitting an occupied voxel.
 * 
 * @param viewpoint Candidate viewpoint
 * @param octomap_interface OctoMap interface for scene representation
 * @param h_fov_rad Camera horizontal field of view (radians)
 * @param v_fov_rad Camera vertical field of view (radians)
 * @param width Image width (pixels)
 * @param height Image height (pixels)
 * @param max_range Maximum sensor range (meters)
 * @param resolution Ray marching step size (meters)
 * @param num_rays Number of rays to cast (-1 for all pixels)
 * @param use_bbox If true, only count voxels within octomap bounding box
 * @return Average unknown voxels per ray
 */
double computeInformationGain(
    const Viewpoint& viewpoint,
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    double h_fov_rad = 1.0472,  // 60 degrees
    double v_fov_rad = 0.7854,  // 45 degrees
    int width = 640,
    int height = 480,
    double max_range = 3.0,
    double resolution = 0.1,
    int num_rays = -1,
    bool use_bbox = true,
    const rclcpp::Logger& logger = rclcpp::get_logger("viewpoint_generation"));

/**
 * @brief Compute semantic information gain for a viewpoint
 * 
 * Combines volumetric uncertainty (unknown voxels) with semantic uncertainty
 * (low confidence in semantic labels).
 * 
 * SIG = (1/|R|) * sum_r [# unknown voxels + beta * sum((1 - confidence))]
 * 
 * @param viewpoint Candidate viewpoint
 * @param octomap_interface OctoMap interface for scene representation
 * @param h_fov_rad Camera horizontal field of view (radians)
 * @param v_fov_rad Camera vertical field of view (radians)
 * @param width Image width (pixels)
 * @param height Image height (pixels)
 * @param max_range Maximum sensor range (meters)
 * @param resolution Ray marching step size (meters)
 * @param num_rays Number of rays to cast (-1 for all pixels)
 * @param use_bbox If true, only count voxels within octomap bounding box
 * @param beta Weight for semantic uncertainty (0=volumetric only)
 * @return Average semantic information gain per ray
 */
double computeSemanticInformationGain(
    const Viewpoint& viewpoint,
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    double h_fov_rad = 1.0472,  // 60 degrees
    double v_fov_rad = 0.7854,  // 45 degrees
    int width = 640,
    int height = 480,
    double max_range = 3.0,
    double resolution = 0.1,
    int num_rays = -1,
    bool use_bbox = true,
    double beta = 1.0,
    bool count_background = true,
    const rclcpp::Logger& logger = rclcpp::get_logger("viewpoint_generation"));

} // namespace erwinia_os_nbv_planner
