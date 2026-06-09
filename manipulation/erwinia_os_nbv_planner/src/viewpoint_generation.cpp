#include "erwinia_os_nbv_planner/viewpoint_generation.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>
#include <random>
#include <iostream>

namespace erwinia_os_nbv_planner
{

    std::optional<std::vector<double>> computeViewpointJointAngles(
        const std::shared_ptr<MoveItInterface> &moveit_interface,
        Viewpoint &viewpoint,
        const std::string &camera_link,
        double ik_timeout,
        int ik_attempts,
        const rclcpp::Logger &logger)
    {
        if (!moveit_interface)
        {
            RCLCPP_ERROR(logger, "MoveItInterface is null");
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 1) Construct camera pose from viewpoint (world frame)
        // ------------------------------------------------------------
        geometry_msgs::msg::Pose cam_pose_world;
        cam_pose_world.position.x = viewpoint.position.x();
        cam_pose_world.position.y = viewpoint.position.y();
        cam_pose_world.position.z = viewpoint.position.z();

        // Assumes viewpoint.orientation = [x, y, z, w]
        cam_pose_world.orientation.x = viewpoint.orientation[0];
        cam_pose_world.orientation.y = viewpoint.orientation[1];
        cam_pose_world.orientation.z = viewpoint.orientation[2];
        cam_pose_world.orientation.w = viewpoint.orientation[3];

        // ------------------------------------------------------------
        // 2) Convert camera pose → end-effector pose
        // ------------------------------------------------------------
        geometry_msgs::msg::Pose target_ee_pose;
        if (!moveit_interface->cameraPoseToEEPose(
                cam_pose_world, camera_link, target_ee_pose))
        {
            RCLCPP_ERROR(logger, "Failed to convert camera pose to end-effector pose (camera_link = %s)", camera_link.c_str());
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 3) Get current joint state as IK seed
        // ------------------------------------------------------------
        std::vector<double> seed_positions;
        if (!moveit_interface->getCurrentJointAngles(seed_positions))
        {
            RCLCPP_ERROR(logger, "Failed to get current joint angles");
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 4) Solve IK
        // ------------------------------------------------------------
        std::vector<double> joint_angles = moveit_interface->computeIK(
            seed_positions,
            target_ee_pose,
            ik_timeout,
            ik_attempts);

        if (joint_angles.empty())
        {
            RCLCPP_WARN(logger, "IK failed for viewpoint at [%.3f, %.3f, %.3f]",
                        viewpoint.position.x(), viewpoint.position.y(), viewpoint.position.z());
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 5) Validate IK solution via FK
        // ------------------------------------------------------------
        Eigen::Isometry3d target_ee_eigen;
        tf2::fromMsg(target_ee_pose, target_ee_eigen);

        double pos_error = 0.0;
        double angular_error = 0.0;
        if (!moveit_interface->validateIKSolution(
                joint_angles,
                target_ee_eigen,
                pos_error,
                angular_error))
        {
            RCLCPP_WARN(logger, "IK solution validation failed. Position error: %.4f m, Angular error: %.2f deg",
                        pos_error, angular_error * 180.0 / M_PI);
        }

        // ------------------------------------------------------------
        // 6) Store solution in viewpoint
        // ------------------------------------------------------------
        viewpoint.joint_angles = joint_angles;

        return joint_angles;
    }

    std::vector<Viewpoint> computeSphericalCap(
        const Eigen::Vector3d &position,
        const std::array<double, 4> &orientation,
        double radius,
        double angular_resolution,
        double max_theta,
        bool look_at_center,
        bool use_positive_z)
    {
        if (radius <= 0.0)
        {
            throw std::invalid_argument("radius must be > 0");
        }
        if (angular_resolution <= 0.0 || max_theta < 0.0)
        {
            throw std::invalid_argument("angular_resolution must be > 0 and max_theta >= 0");
        }

        std::vector<Viewpoint> viewpoints;

        const Eigen::Quaterniond base_quat = geometry_utils::arrayToEigenQuat(orientation);
        const Eigen::Matrix3d R_base = base_quat.toRotationMatrix();
        const Eigen::Vector3d cap_x = R_base.col(0);
        const Eigen::Vector3d cap_y = R_base.col(1);
        const Eigen::Vector3d cap_z = R_base.col(2);

        const double z_sign = use_positive_z ? 1.0 : -1.0;
        const Eigen::Vector3d cap_axis = z_sign * cap_z; // <-- FIX

        std::vector<double> thetas = {0.0};
        for (double t = angular_resolution; t <= max_theta + 1e-12; t += angular_resolution)
        {
            thetas.push_back(t);
        }

        for (size_t ring_idx = 0; ring_idx < thetas.size(); ++ring_idx)
        {
            const double theta = thetas[ring_idx];

            if (theta == 0.0)
            {
                const Eigen::Vector3d local(0.0, 0.0, z_sign * radius);
                const Eigen::Vector3d p = position + local.x() * cap_x + local.y() * cap_y + local.z() * cap_z;

                Eigen::Vector3d direction = look_at_center ? (position - p) : (p - position);
                const double n = direction.norm();
                if (n < 1e-12)
                    continue;
                direction /= n;

                const auto quat_delta = geometry_utils::quatFromTwoVectors(cap_axis, direction); // <-- FIX
                const Eigen::Quaterniond q_delta = geometry_utils::arrayToEigenQuat(quat_delta);
                const Eigen::Quaterniond q_world = q_delta * base_quat;

                Viewpoint vp(p, geometry_utils::eigenQuatToArray(q_world));
                vp.target = position;
                viewpoints.push_back(vp);
                continue;
            }

            const double ring_circum = 2.0 * M_PI * std::sin(theta);
            const int m = std::max(1, static_cast<int>(std::ceil(ring_circum / angular_resolution)));
            const double phi_offset = (ring_idx % 2 == 1) ? (M_PI / m) : 0.0;

            for (int k = 0; k < m; ++k)
            {
                const double phi = (2.0 * M_PI * k) / m + phi_offset;

                const double local_x = radius * std::sin(theta) * std::cos(phi);
                const double local_y = radius * std::sin(theta) * std::sin(phi);
                const double local_z = z_sign * radius * std::cos(theta);

                const Eigen::Vector3d p = position + local_x * cap_x + local_y * cap_y + local_z * cap_z;

                Eigen::Vector3d direction = look_at_center ? (position - p) : (p - position);
                const double n = direction.norm();
                if (n < 1e-12)
                    continue;
                direction /= n;

                const auto quat_delta = geometry_utils::quatFromTwoVectors(cap_axis, direction); // <-- FIX
                const Eigen::Quaterniond q_delta = geometry_utils::arrayToEigenQuat(quat_delta);
                const Eigen::Quaterniond q_world = q_delta * base_quat;

                Viewpoint vp(p, geometry_utils::eigenQuatToArray(q_world));
                vp.target = position;
                viewpoints.push_back(vp);
            }
        }

        return viewpoints;
    }

    std::vector<Viewpoint> generatePlanarSphericalCapCandidates(
        const Eigen::Vector3d &position,
        const std::array<double, 4> &orientation,
        double half_extent,
        double spatial_resolution,
        double max_theta,
        double angular_resolution)
    {
        // Extract basis vectors from orientation
        const Eigen::Quaterniond base_quat = geometry_utils::arrayToEigenQuat(orientation);
        const Eigen::Matrix3d R_base = base_quat.toRotationMatrix();
        const Eigen::Vector3d plane_x = R_base.col(0);
        const Eigen::Vector3d plane_y = R_base.col(1);
        const Eigen::Vector3d normal = R_base.col(2);

        // Generate grid of positions on the plane
        std::vector<Eigen::Vector3d> grid_positions;
        for (double x = -half_extent; x <= half_extent + spatial_resolution / 2; x += spatial_resolution)
        {
            for (double y = -half_extent; y <= half_extent + spatial_resolution / 2; y += spatial_resolution)
            {
                const Eigen::Vector3d pos = position + x * plane_x + y * plane_y;
                grid_positions.push_back(pos);
            }
        }

        // Generate orientation variations
        std::vector<std::array<double, 4>> orientations;

        if (max_theta == 0 || angular_resolution == 0)
        {
            // No variations, just use base orientation
            orientations.push_back(orientation);
        }
        else
        {
            // Get orientations from spherical cap (dummy position)
            const auto cap_viewpoints = computeSphericalCap(
                Eigen::Vector3d::Zero(), orientation, 1.0,
                angular_resolution, max_theta, false, true);

            for (const auto &vp : cap_viewpoints)
            {
                orientations.push_back(vp.orientation);
            }
        }

        // Create viewpoints for each position with each orientation
        std::vector<Viewpoint> viewpoints;
        for (const auto &pos : grid_positions)
        {
            for (const auto &orient : orientations)
            {
                Viewpoint vp(pos, orient);
                vp.target = pos + normal;
                viewpoints.push_back(vp);
            }
        }

        return viewpoints;
    }

    std::vector<Viewpoint> sampleViewsFromHemisphere(
        const Eigen::Vector3d &center,
        const std::array<double, 4> &base_orientation,
        double min_radius,
        double max_radius,
        int num_samples,
        bool use_positive_z,
        double z_bias_sigma,
        double min_distance,
        int max_attempts,
        const rclcpp::Logger &logger)
    {
        if (min_radius < 0)
        {
            throw std::invalid_argument("min_radius must be non-negative");
        }
        if (max_radius < min_radius)
        {
            throw std::invalid_argument("max_radius must be >= min_radius");
        }
        if (num_samples <= 0)
        {
            throw std::invalid_argument("num_samples must be positive");
        }
        if (z_bias_sigma <= 0)
        {
            throw std::invalid_argument("z_bias_sigma must be positive");
        }
        if (min_distance < 0)
        {
            throw std::invalid_argument("min_distance must be non-negative");
        }

        const bool sample_surface = std::abs(max_radius - min_radius) < 1e-9;

        // Extract basis vectors
        const Eigen::Quaterniond base_quat = geometry_utils::arrayToEigenQuat(base_orientation);
        const Eigen::Matrix3d R_base = base_quat.toRotationMatrix();
        const Eigen::Vector3d cap_x = R_base.col(0);
        const Eigen::Vector3d cap_y = R_base.col(1);
        const Eigen::Vector3d cap_z = R_base.col(2);

        const double z_sign = use_positive_z ? 1.0 : -1.0;

        // Random number generators (fixed seed for reproducibility)
        std::mt19937 gen(42u);
        std::uniform_real_distribution<> uniform_01(0.0, 1.0);
        std::uniform_real_distribution<> uniform_phi(0.0, 2.0 * M_PI);
        std::normal_distribution<> gaussian_theta(0.0, z_bias_sigma);

        std::vector<Viewpoint> viewpoints;
        std::vector<Eigen::Vector3d> positions;

        int attempts = 0;
        while (viewpoints.size() < static_cast<size_t>(num_samples) && attempts < max_attempts)
        {
            attempts++;

            // Sample radius
            double r;
            if (sample_surface)
            {
                r = min_radius;
            }
            else
            {
                const double u = uniform_01(gen);
                const double r_cubed = min_radius * min_radius * min_radius +
                                       u * (max_radius * max_radius * max_radius - min_radius * min_radius * min_radius);
                r = std::cbrt(r_cubed);
            }

            // Gaussian sampling for theta, clamped to [0, pi/2]
            double theta = std::abs(gaussian_theta(gen));
            theta = std::min(theta, M_PI / 2.0);

            // Uniform azimuthal angle
            const double phi = uniform_phi(gen);

            // Convert to Cartesian in local frame
            const double local_x = r * std::sin(theta) * std::cos(phi);
            const double local_y = r * std::sin(theta) * std::sin(phi);
            const double local_z = z_sign * r * std::cos(theta);

            // Transform to world
            const Eigen::Vector3d pos = center +
                                        local_x * cap_x + local_y * cap_y + local_z * cap_z;

            // Check minimum distance constraint
            if (min_distance > 0 && !positions.empty())
            {
                bool too_close = false;
                for (const auto &existing_pos : positions)
                {
                    if ((existing_pos - pos).norm() < min_distance)
                    {
                        too_close = true;
                        break;
                    }
                }
                if (too_close)
                    continue;
            }

            // Accept this point
            positions.push_back(pos);

            // Compute orientation facing toward center
            Eigen::Vector3d direction = center - pos;
            direction.normalize();

            const auto quat_delta = geometry_utils::quatFromTwoVectors(cap_z, direction);
            const Eigen::Quaterniond q_delta = geometry_utils::arrayToEigenQuat(quat_delta);
            const Eigen::Quaterniond q_world = q_delta * base_quat;

            Viewpoint vp(pos, geometry_utils::eigenQuatToArray(q_world));
            vp.target = center;
            viewpoints.push_back(vp);
        }

        if (viewpoints.size() < static_cast<size_t>(num_samples))
        {
            RCLCPP_WARN(logger, "Only generated %zu/%d viewpoints. Try reducing min_distance or increasing the sampling region.",
                        viewpoints.size(), num_samples);
        }

        return viewpoints;
    }

    std::vector<Viewpoint> generateFrontierBasedViewpoints(
        const std::vector<Eigen::Vector3d> &centers,
        const std::array<double, 4> &base_orientation,
        double min_radius,
        double max_radius,
        int num_samples,
        bool use_positive_z,
        double z_bias_sigma,
        double min_distance,
        int max_attempts,
        const rclcpp::Logger &logger)
    {
        std::vector<Viewpoint> viewpoints;
        for (const auto &center : centers)
        {
            std::vector<Viewpoint> center_viewpoints = sampleViewsFromHemisphere(
                center,
                base_orientation,
                min_radius,
                max_radius,
                num_samples,
                use_positive_z,
                z_bias_sigma,
                min_distance,
                max_attempts,
                logger);
            viewpoints.insert(viewpoints.end(), center_viewpoints.begin(), center_viewpoints.end());
        }
        return viewpoints;
    }

    std::pair<std::vector<Eigen::Vector3d>, Eigen::Vector3d> computePlane(
        const std::shared_ptr<OctoMapInterface> &octomap_interface,
        const Eigen::Vector3d &cam_position)
    {
        // Compute the midpoint of the bounding box
        octomap::point3d min_octo, max_octo;
        octomap_interface->getBoundingBox(min_octo, max_octo);
        Eigen::Vector3d min_bbox = conversions::octomapToEigen(min_octo);
        Eigen::Vector3d max_bbox = conversions::octomapToEigen(max_octo);
        Eigen::Vector3d mid = 0.5 * (min_bbox + max_bbox);

        // Compute the viewing direction
        Eigen::Vector3d distance = (cam_position - mid);
        Eigen::Vector3d view_dir = distance.normalized();
        // If the viewing direction is aligned with the x axis, then we are using the YZ plane
        Eigen::Vector3d abs_view_dir = view_dir.cwiseAbs();
        if (abs_view_dir.x() >= abs_view_dir.y()) // YZ plane
        {
            std::vector<Eigen::Vector3d> corners = {
                {mid.x(), min_bbox.y(), min_bbox.z()},
                {mid.x(), max_bbox.y(), min_bbox.z()},
                {mid.x(), max_bbox.y(), max_bbox.z()},
                {mid.x(), min_bbox.y(), max_bbox.z()},
            };
            return std::make_pair(corners, distance);
        }
        else // XZ plane
        {
            std::vector<Eigen::Vector3d> corners = {
                {min_bbox.x(), mid.y(), min_bbox.z()},
                {max_bbox.x(), mid.y(), min_bbox.z()},
                {max_bbox.x(), mid.y(), max_bbox.z()},
                {min_bbox.x(), mid.y(), max_bbox.z()},
            };
            return std::make_pair(corners, distance);
        }
    }

    std::pair<std::vector<Viewpoint>, std::vector<std::vector<Eigen::Vector3d>>> generateViewpointsFromPlane(
        const std::vector<Eigen::Vector3d> &corners,
        const Eigen::Vector3d &distance,
        const Eigen::Quaterniond &orientation_base,
        double overlap_ratio,
        double camera_hfov_rad, 
        double camera_vfov_rad)
    {
        // Determine the midpoint of the plane
        Eigen::Vector3d mid = 0.25 * (corners[0] + corners[1] + corners[2] + corners[3]);

        // Determine which plane we are using (YZ or XZ) based on the corners
        bool is_yz_plane = (std::abs(corners[0].x() - corners[1].x()) < 1e-6);

        // Calculate the coverage area at the given distance (just along the x or y direction)
        double u_coverage, v_coverage;
        if (is_yz_plane) // YZ plane
        {
            u_coverage = 2.0 * distance.x() * std::tan(camera_hfov_rad / 2.0);
            v_coverage = 2.0 * distance.x() * std::tan(camera_vfov_rad / 2.0);
        }
        else // XZ plane
        {
            u_coverage = 2.0 * distance.y() * std::tan(camera_hfov_rad / 2.0);
            v_coverage = 2.0 * distance.y() * std::tan(camera_vfov_rad / 2.0);
        }

        // Once we have the coverage, we can generate a grid of viewpoints on the plane
        double spacing_u = u_coverage * (1.0 - overlap_ratio);
        double spacing_v = v_coverage * (1.0 - overlap_ratio);

        // Starting from the center of the plane, generate grid points
        std::vector<Eigen::Vector3d> viewpoint_positions;
        std::vector<std::vector<Eigen::Vector3d>> coverage_planes;
        int num_u = 0, num_v = 0;
        if (is_yz_plane)
        {
            // YZ plane at constant x
            double plane_height = corners[2].z() - corners[0].z();
            double plane_width = corners[1].y() - corners[0].y();
            // Compute number of viewpoints along each axis
            num_u = std::max(1, static_cast<int>(std::ceil(plane_width / spacing_u)));
            num_v = std::max(1, static_cast<int>(std::ceil(plane_height / spacing_v)));
            // Generate grid points
            double start_y = mid.y() - (num_u - 1) * spacing_u / 2.0;
            double start_z = mid.z() - (num_v - 1) * spacing_v / 2.0;
            for (int i = 0; i < num_u; ++i)
            {
                for (int j = 0; j < num_v; ++j)
                {
                    viewpoint_positions.push_back(
                        Eigen::Vector3d(distance.x() + mid.x(), // x is from distance
                                        start_y + i * spacing_u,
                                        start_z + j * spacing_v));
                    coverage_planes.push_back({
                        Eigen::Vector3d(mid.x(),
                                        start_y + i * spacing_u - u_coverage / 2.0,
                                        start_z + j * spacing_v - v_coverage / 2.0),
                        Eigen::Vector3d(mid.x(),
                                        start_y + i * spacing_u + u_coverage / 2.0,
                                        start_z + j * spacing_v - v_coverage / 2.0),
                        Eigen::Vector3d(mid.x(),
                                        start_y + i * spacing_u + u_coverage / 2.0,
                                        start_z + j * spacing_v + v_coverage / 2.0),
                        Eigen::Vector3d(mid.x(),
                                        start_y + i * spacing_u - u_coverage / 2.0,
                                        start_z + j * spacing_v + v_coverage / 2.0),
                    });
                }
            }
        }
        else // XZ plane
        {
            double plane_height = corners[2].z() - corners[0].z();
            double plane_width = corners[1].x() - corners[0].x();
            num_u = std::max(1, static_cast<int>(std::ceil(plane_width / spacing_u)));
            num_v = std::max(1, static_cast<int>(std::ceil(plane_height / spacing_v)));
            double start_x = mid.x() - (num_u - 1) * spacing_u / 2.0;
            double start_z = mid.z() - (num_v - 1) * spacing_v / 2.0;
            for (int i = 0; i < num_u; ++i)
            {
                for (int j = 0; j < num_v; ++j)
                {
                    viewpoint_positions.push_back(
                        Eigen::Vector3d(start_x + i * spacing_u,
                                        distance.y() + mid.y(), // y is from distance
                                        start_z + j * spacing_v));
                    coverage_planes.push_back({
                        Eigen::Vector3d(start_x + i * spacing_u - u_coverage / 2.0,
                                        mid.y(),
                                        start_z + j * spacing_v - v_coverage / 2.0),
                        Eigen::Vector3d(start_x + i * spacing_u + u_coverage / 2.0,
                                        mid.y(),
                                        start_z + j * spacing_v - v_coverage / 2.0),
                        Eigen::Vector3d(start_x + i * spacing_u + u_coverage / 2.0,
                                        mid.y(),
                                        start_z + j * spacing_v + v_coverage / 2.0),
                        Eigen::Vector3d(start_x + i * spacing_u - u_coverage / 2.0,
                                        mid.y(),
                                        start_z + j * spacing_v + v_coverage / 2.0),
                    });
                }
            }
        }

        // Create the viewpoints with the computed positions and base orientation
        std::vector<Viewpoint> viewpoints;
        for (const auto &pos : viewpoint_positions)
        {
            Viewpoint vp;
            vp.position = pos;
            vp.orientation = {orientation_base.x(), orientation_base.y(),
                            orientation_base.z(), orientation_base.w()};
            viewpoints.push_back(vp);
        }

        return std::make_pair(viewpoints, coverage_planes);
    }

    std::vector<Viewpoint> generateSphericalCaps(
        const std::vector<Viewpoint> &grid_viewpoints,
        const std::array<double, 4> &orientation,
        double max_theta,
        double angular_resolution)
    {
        // Generate orientation variations
        std::vector<std::array<double, 4>> orientations;
        Eigen::Vector3d normal = geometry_utils::quatToRotMat(orientation).col(2);

        if (max_theta == 0 || angular_resolution == 0)
        {
            // No variations, just use base orientation
            orientations.push_back(orientation);
        }
        else
        {
            // Get orientations from spherical cap (dummy position)
            const auto cap_viewpoints = computeSphericalCap(
                Eigen::Vector3d::Zero(), orientation, 1.0,
                angular_resolution, max_theta, false, true);

            for (const auto &vp : cap_viewpoints)
            {
                orientations.push_back(vp.orientation);
            }
        }

        // Create viewpoints for each position with each orientation
        std::vector<Viewpoint> viewpoints;
        for (const auto &pos : grid_viewpoints)
        {
            for (const auto &orient : orientations)
            {
                Viewpoint vp(pos.position, orient);
                vp.target = pos.position + normal;
                viewpoints.push_back(vp);
            }
        }
    
        return viewpoints;
    }

    double computeInformationGain(
        const Viewpoint &viewpoint,
        const std::shared_ptr<OctoMapInterface> &octomap_interface,
        double h_fov_rad,
        double v_fov_rad,
        int width,
        int height,
        double max_range,
        double resolution,
        int num_rays,
        bool use_bbox,
        const rclcpp::Logger &logger)
    {
        if (!octomap_interface || width <= 0 || height <= 0 || max_range <= 0 || resolution <= 0)
        {
            return 0.0;
        }

        // Check if tree is available
        if (!octomap_interface->isTreeAvailable())
        {
            RCLCPP_ERROR(logger, "Octomap tree is not available");
            return 0.0;
        }

        // Get bounding box if needed
        octomap::point3d bbox_min, bbox_max;
        if (use_bbox)
        {
            if (!octomap_interface->getBoundingBox(bbox_min, bbox_max))
            {
                RCLCPP_ERROR(logger, "Failed to get bounding box from octomap");
                return 0.0;
            }
        }

        // // Please display the bounding box in RViz for debugging
        // if (visualizer)
        // {
        //     visualizer->publishBoundingBox(bbox_min, bbox_max);
        // }

        // Get octree depth
        const int max_depth = octomap_interface->getOctreeDepth();
        if (max_depth == 0)
        {
            return 0.0;
        }

        // Compute ray strides
        const auto [stride_u, stride_v] = geometry_utils::computeRayStrides(width, height, num_rays);

        // Camera extrinsics
        const Eigen::Matrix3d R_wc = geometry_utils::quatToRotMat(viewpoint.orientation);

        // Camera intrinsics
        const double focal_x = (width / 2.0) / std::tan(h_fov_rad / 2.0);
        const double focal_y = (height / 2.0) / std::tan(v_fov_rad / 2.0);
        const double cx = (width - 1) * 0.5;
        const double cy = (height - 1) * 0.5;

        int total_unknown = 0;
        int num_rays_cast = 0;
        // int point_id = 0; // Counter for unique point marker IDs

        for (int v = 0; v < height; v += stride_v)
        {
            for (int u = 0; u < width; u += stride_u)
            {
                // Ray in camera frame
                const double x = (u - cx) / focal_x;
                const double y = (v - cy) / focal_y;
                Eigen::Vector3d ray_cam(x, y, 1.0);
                ray_cam.normalize();

                // Transform to world frame
                Eigen::Vector3d ray_world = R_wc * ray_cam;
                ray_world.normalize();

                // March along the ray
                int unknown_count = 0;
                bool prev_point_in_bbox = false;

                const int num_steps = static_cast<int>(max_range / resolution);
                for (int i = 0; i < num_steps; ++i)
                {
                    const double t = i * resolution;
                    const Eigen::Vector3d point = viewpoint.position + t * ray_world;
                    const octomap::point3d oct_point(point.x(), point.y(), point.z());

                    // Check bounding box if enabled
                    bool current_in_bbox = true;
                    if (use_bbox)
                    {
                        current_in_bbox = (oct_point.x() >= bbox_min.x() && oct_point.x() <= bbox_max.x() &&
                                           oct_point.y() >= bbox_min.y() && oct_point.y() <= bbox_max.y() &&
                                           oct_point.z() >= bbox_min.z() && oct_point.z() <= bbox_max.z());

                        // End ray if exiting bbox
                        if (!current_in_bbox && prev_point_in_bbox)
                        {
                            break;
                        }
                        prev_point_in_bbox = current_in_bbox;
                    }

                    // Determine voxel state and color for visualization
                    std_msgs::msg::ColorRGBA color;
                    color.a = 0.8f;
                    bool should_break = false;

                    if (!current_in_bbox)
                    {
                        // Gray for outside bounding box
                        color.r = 0.5f;
                        color.g = 0.5f;
                        color.b = 0.5f;
                    }

                    if (current_in_bbox)
                    {
                        // Inside bbox - check voxel state
                        octomap::OcTreeNode *node = nullptr;
                        const bool found = octomap_interface->searchOctree(oct_point, node);

                        if (!found || node == nullptr)
                        {
                            // Unknown voxel - GREEN
                            unknown_count++;
                            color.r = 0.0f;
                            color.g = 1.0f;
                            color.b = 0.0f;
                        }
                        else if (octomap_interface->isVoxelOccupied(oct_point))
                        {
                            // Occupied voxel - BLUE (will stop ray after visualization)
                            color.r = 0.0f;
                            color.g = 0.0f;
                            color.b = 1.0f;
                            should_break = true;
                        }
                        else
                        {
                            // Free voxel - YELLOW
                            color.r = 1.0f;
                            color.g = 1.0f;
                            color.b = 0.0f;
                        }
                    }

                    // if (visualizer)
                    // {
                    //     visualizer->publishPoint(oct_point, point_id++, 0.015, color, 0.8f, "ig_rays");
                    //     // Small delay to allow RViz to process the marker
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    // }

                    // Stop ray if hit occupied voxel
                    if (should_break)
                        break;
                }

                total_unknown += unknown_count;
                num_rays_cast++;
            }
        }

        return num_rays_cast > 0 ? static_cast<double>(total_unknown) / num_rays_cast : 0.0;
    }

    double computeSemanticInformationGain(
        const Viewpoint &viewpoint,
        const std::shared_ptr<OctoMapInterface> &octomap_interface,
        double h_fov_rad,
        double v_fov_rad,
        int width,
        int height,
        double max_range,
        double resolution,
        int num_rays,
        bool use_bbox,
        double beta,
        bool count_background,
        const rclcpp::Logger &logger)
    {
        if (!octomap_interface || width <= 0 || height <= 0 || max_range <= 0 || resolution <= 0)
            return 0.0;

        // Must have a semantic tree
        auto sem_tree = octomap_interface->getSemanticTree();
        if (!sem_tree)
        {
            RCLCPP_WARN(logger, "Octomap semantic tree is not available, cannot compute semantic IG");
            // Fallback: just volumetric IG
            return computeInformationGain(
                viewpoint, octomap_interface,
                h_fov_rad, v_fov_rad,
                width, height,
                max_range, resolution,
                num_rays, use_bbox,
                logger);
        }

        // Bounding box
        octomap::point3d bbox_min, bbox_max;
        if (use_bbox)
        {
            if (!octomap_interface->getBoundingBox(bbox_min, bbox_max))
                return 0.0;
        }

        // Ray strides
        const auto [stride_u, stride_v] = geometry_utils::computeRayStrides(width, height, num_rays);

        // Camera extrinsics (world <- cam)
        const Eigen::Matrix3d R_wc = geometry_utils::quatToRotMat(viewpoint.orientation);

        // Camera intrinsics (pinhole from FOV)
        const double focal_x = (width  / 2.0) / std::tan(h_fov_rad / 2.0);
        const double focal_y = (height / 2.0) / std::tan(v_fov_rad / 2.0);
        const double cx = (width  - 1) * 0.5;
        const double cy = (height - 1) * 0.5;

        auto in_bbox = [&](const octomap::point3d& p) -> bool {
            if (!use_bbox) return true;
            return (p.x() >= bbox_min.x() && p.x() <= bbox_max.x() &&
                    p.y() >= bbox_min.y() && p.y() <= bbox_max.y() &&
                    p.z() >= bbox_min.z() && p.z() <= bbox_max.z());
        };

        double total_gain = 0.0;
        int num_rays_cast = 0;

        const int num_steps  = static_cast<int>(max_range / resolution);
        const double t_peak  = max_range / 2.0;
        const double t_sigma = max_range / 4.0;

        for (int v = 0; v < height; v += stride_v)
        {
            for (int u = 0; u < width; u += stride_u)
            {
                // Ray in camera frame (z forward)
                const double x = (u - cx) / focal_x;
                const double y = (v - cy) / focal_y;
                Eigen::Vector3d ray_cam(x, y, 1.0);
                ray_cam.normalize();

                // Transform to world
                Eigen::Vector3d ray_world = R_wc * ray_cam;
                ray_world.normalize();

                double ray_gain = 0.0;
                bool prev_in_bbox = false;

                for (int i = 0; i < num_steps; ++i)
                {
                    const double t = i * resolution;
                    const Eigen::Vector3d point = viewpoint.position + t * ray_world;
                    const octomap::point3d oct_point(point.x(), point.y(), point.z());

                    const bool cur_in_bbox = in_bbox(oct_point);

                    // Stop the ray once we leave bbox after being inside (single convex region)
                    if (use_bbox && !cur_in_bbox && prev_in_bbox)
                        break;
                    prev_in_bbox = cur_in_bbox;

                    if (use_bbox && !cur_in_bbox)
                        continue;

                    // Semantic tree search
                    octomap::SemanticOcTreeNode* node = sem_tree->search(oct_point);

                    if (!node)
                    {
                        // Unknown voxel => volumetric uncertainty weighted by 1
                        ray_gain += (1.0 - static_cast<double>(beta));
                        continue;
                    }

                    // Known voxel:
                    if (sem_tree->isNodeOccupied(node))
                    {
                        // Occupied => semantic uncertainty term
                        if (node->isSemanticSet() || count_background)
                        {
                            float conf = node->getConfidence();
                            if (std::isnan(conf) || std::isinf(conf)) conf = 0.0f;
                            conf = std::min(1.0f, std::max(0.0f, conf));
                            // const double dist_weight = std::exp(-0.5 * std::pow((t - t_peak) / t_sigma, 2.0));
                            const double dist_weight = 1.0; // No distance weighting for now
                            ray_gain += static_cast<double>(beta) * static_cast<double>(num_steps) * (1.0 - static_cast<double>(conf)) * dist_weight;
                        }
                        // else { ray_gain += static_cast<double>(beta) * static_cast<double>(num_steps); } // No semantic info => max uncertainty

                        // Stop ray at occupied voxel
                        break;
                    }

                    // Free voxel => nothing, keep marching
                }

                total_gain += ray_gain;
                num_rays_cast++;
            }
        }

        return (num_rays_cast > 0) ? (total_gain / static_cast<double>(num_rays_cast)) : 0.0;
    }

} // namespace erwinia_os_nbv_planner
