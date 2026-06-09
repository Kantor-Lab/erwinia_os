#include "erwinia_os_nbv_planner/moveit_interface.hpp"

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <random>
#include <ctime>

namespace erwinia_os_nbv_planner
{

    MoveItInterface::MoveItInterface(const std::shared_ptr<rclcpp::Node> &node,
                                     const std::string &group_name)
        : node_(node), group_name_(group_name)
    {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, group_name_);
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");

        if (!isPSMValid())
            return;

        psm_->startSceneMonitor();
        psm_->startStateMonitor();
        psm_->startWorldGeometryMonitor();

        RCLCPP_DEBUG(node_->get_logger(), "Waiting for current robot state...");
        if (!psm_->waitForCurrentRobotState(node_->now(), 5.0))
        {
            RCLCPP_WARN(node_->get_logger(), "Timed out waiting for robot state.");
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(), "Robot state received successfully.");
        }

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        auto world_objects = scene->getWorld()->getObjectIds();
        RCLCPP_DEBUG(node_->get_logger(), "Planning scene has %zu world objects", world_objects.size());
        for (const auto &obj_id : world_objects)
        {
            RCLCPP_DEBUG(node_->get_logger(), "  - %s", obj_id.c_str());
        }

        // Initialize cached configuration with MoveGroup defaults
        if (isMoveGroupValid(false))
        {
            pose_reference_frame_ = move_group_->getPoseReferenceFrame();
            end_effector_link_ = move_group_->getEndEffectorLink();
            planning_pipeline_id_ = move_group_->getPlanningPipelineId();
            planner_id_ = move_group_->getPlannerId();
            planning_time_ = move_group_->getPlanningTime();

            RCLCPP_DEBUG(node_->get_logger(), "Initialized config - Pipeline: '%s', Planner: '%s'",
                         planning_pipeline_id_.c_str(), planner_id_.c_str());

            // Set default planner if not already configured
            if (planner_id_.empty())
                move_group_->setPlannerId(planner_id_);

            // Set our default values
            move_group_->setNumPlanningAttempts(num_planning_attempts_);
            move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
            move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
        }
        
        // Initialize default reference frame from robot model
        if (isPSMValid(false))
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
            const moveit::core::RobotModelConstPtr &robot_model = scene->getRobotModel();
            if (robot_model)
            {
                pose_reference_frame_ = robot_model->getRootLinkName();
                RCLCPP_DEBUG(node_->get_logger(), "Pose reference frame set to robot root link: '%s'",
                           pose_reference_frame_.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Could not get robot model, pose reference frame not set");
            }
        }
    }

    bool MoveItInterface::isPSMValid(bool verbose) const
    {
        bool valid = (psm_ && psm_->getPlanningScene());
        if (!valid && verbose)
            RCLCPP_ERROR(node_->get_logger(), "PlanningSceneMonitor is NOT valid.");
        return valid;
    }

    bool MoveItInterface::isMoveGroupValid(bool verbose) const
    {
        bool valid = (move_group_ != nullptr);
        if (!valid && verbose)
            RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is NOT initialized.");
        return valid;
    }

    bool MoveItInterface::getJointModelGroup(moveit::core::JointModelGroupConstPtr &jmg_out,
                                             const moveit::core::RobotModelConstPtr &robot_model)
    {
        if (!robot_model)
        {
            RCLCPP_ERROR(node_->get_logger(), "RobotModel is null");
            return false;
        }

        const moveit::core::JointModelGroup *jmg_raw = robot_model->getJointModelGroup(group_name_);
        if (!jmg_raw)
        {
            RCLCPP_ERROR(node_->get_logger(), "JointModelGroup '%s' not found.", group_name_.c_str());
            return false;
        }

        // Store raw pointer in shared_ptr with no-op deleter since robot_model owns it
        jmg_out = moveit::core::JointModelGroupConstPtr(jmg_raw, [](const moveit::core::JointModelGroup *) {});
        return true;
    }

    bool MoveItInterface::getCurrentJointAngles(std::vector<double> &joints_out,
                                                const moveit::core::JointModelGroupConstPtr &jmg)
    {
        if (!isPSMValid())
            return false;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &current_state = scene->getCurrentState();
        current_state.copyJointGroupPositions(jmg.get(), joints_out);
        return true;
    }

    bool MoveItInterface::getCurrentJointAngles(std::vector<double> &joints_out)
    {
        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg))
            return false;
        return getCurrentJointAngles(joints_out, jmg);
    }

    bool MoveItInterface::validateJointPositions(const std::vector<double> &joint_positions)
    {
        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg))
            return false;

        const std::vector<std::string> &active_joint_names = jmg->getActiveJointModelNames();
        if (joint_positions.size() != active_joint_names.size())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "joint_positions size %zu != expected %zu for group '%s'.",
                         joint_positions.size(), active_joint_names.size(), group_name_.c_str());
            return false;
        }
        return true;
    }

    bool MoveItInterface::isStateValid(const std::vector<double> &joint_positions,
                                       planning_scene_monitor::LockedPlanningSceneRO &scene,
                                       moveit::core::RobotState &state)
    {
        if (!isPSMValid())
            return false;

        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg, state.getRobotModel()))
            return false;

        if (!validateJointPositions(joint_positions))
            return false;

        state.setJointGroupPositions(jmg.get(), joint_positions);
        state.update();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.group_name = group_name_;
        scene->checkCollision(req, res, state);

        if (res.collision)
            RCLCPP_DEBUG(node_->get_logger(), "State is in collision");

        return !res.collision;
    }

    bool MoveItInterface::planToJointGoal(const std::vector<double> &joint_positions,
                                          moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        if (!isMoveGroupValid() || !validateJointPositions(joint_positions))
            return false;

        move_group_->setStartStateToCurrentState();
        move_group_->setJointValueTarget(joint_positions);
        moveit::core::MoveItErrorCode result = move_group_->plan(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Planning succeeded. Trajectory: %zu waypoints, %.2fs",
                        plan.trajectory_.joint_trajectory.points.size(),
                        rclcpp::Duration(plan.trajectory_.joint_trajectory.points.back().time_from_start).seconds());
            return true;
        }

        RCLCPP_WARN(node_->get_logger(), "Planning failed (code: %d)", result.val);
        return false;
    }

    bool MoveItInterface::execute(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        if (!isMoveGroupValid())
            return false;

        RCLCPP_DEBUG(node_->get_logger(), "Executing trajectory...");
        moveit::core::MoveItErrorCode result = move_group_->execute(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Trajectory execution succeeded.");
            return true;
        }

        RCLCPP_ERROR(node_->get_logger(), "Trajectory execution failed (code: %d)", result.val);
        return false;
    }

    bool MoveItInterface::planAndExecute(const std::vector<double> &joint_positions)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return planToJointGoal(joint_positions, plan) && execute(plan);
    }

    bool MoveItInterface::planToJointStateWithRetries(const std::vector<double> &joint_positions,
                                                      int num_attempts,
                                                      int plans_per_attempt)
    {
        if (!validateJointPositions(joint_positions))
            return false;

        const double dt_sec = 0.02;
        bool found_valid_plan = false;
        double best_cost = std::numeric_limits<double>::infinity();
        moveit::planning_interface::MoveGroupInterface::Plan best_plan;

        RCLCPP_DEBUG(node_->get_logger(),
                    "Planning to joint state with %d attempts x %d plans",
                    num_attempts, plans_per_attempt);

        for (int attempt = 0; attempt < num_attempts; ++attempt)
        {
            for (int p = 0; p < plans_per_attempt; ++p)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (!planToJointGoal(joint_positions, plan) ||
                    plan.trajectory_.joint_trajectory.points.empty())
                    continue;

                ensureStrictlyIncreasingTime(plan.trajectory_.joint_trajectory, dt_sec);
                double cost = computeJointSpacePathLength(plan.trajectory_.joint_trajectory);

                if (!found_valid_plan || cost < best_cost)
                {
                    best_cost = cost;
                    best_plan = plan;
                    found_valid_plan = true;
                    RCLCPP_DEBUG(node_->get_logger(),
                                "Joint state plan: attempt %d/%d cost=%.3f",
                                attempt, p, cost);
                }
            }

            if (found_valid_plan && best_cost < 2.0)
            {
                RCLCPP_DEBUG(node_->get_logger(), "Found sufficiently good joint plan early, stopping search");
                break;
            }
        }

        if (!found_valid_plan)
        {
            RCLCPP_WARN(node_->get_logger(),
                       "No valid plan found to joint state after %d attempts", num_attempts);
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                   "Executing best joint state plan (cost=%.3f)", best_cost);
        return execute(best_plan);
    }

    std::vector<double> MoveItInterface::computeIK(const std::vector<double> &seed_positions,
                                                   const geometry_msgs::msg::Pose &target_pose,
                                                   double timeout, int attempts,
                                                   const std::string &reference_frame)
    {
        std::vector<double> ik_solution;

        if (!isPSMValid() || !validateJointPositions(seed_positions))
            return ik_solution;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        moveit::core::RobotState state = scene->getCurrentState();

        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg, state.getRobotModel()))
            return ik_solution;

        state.setJointGroupPositions(jmg.get(), seed_positions);
        state.update();

        // Convert target pose to Eigen
        Eigen::Isometry3d target_eigen;
        tf2::fromMsg(target_pose, target_eigen);
        
        // Determine the reference frame to use
        std::string ref_frame = reference_frame.empty() ? pose_reference_frame_ : reference_frame;
        
        // If a specific reference frame is provided, transform the target pose
        if (!ref_frame.empty() && state.knowsFrameTransform(ref_frame))
        {
            const Eigen::Isometry3d T_world_ref = state.getGlobalLinkTransform(ref_frame);
            target_eigen = T_world_ref * target_eigen;
        }

        const std::string &ee_link = jmg->getLinkModelNames().back();

        bool ik_success = false;
        while (attempts > 0 && !ik_success)
        {
            ik_success = state.setFromIK(jmg.get(), target_eigen, ee_link, timeout);
            attempts--;
        }

        if (ik_success)
        {
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            req.group_name = group_name_;
            scene->checkCollision(req, res, state);

            if (!res.collision)
            {
                state.copyJointGroupPositions(jmg.get(), ik_solution);
                RCLCPP_DEBUG(node_->get_logger(), "IK solution found (collision-free) for frame '%s'.", 
                            ref_frame.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "IK solution found but in collision.");
            }
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(), "IK solution not found for frame '%s'.", ref_frame.c_str());
        }

        return ik_solution;
    }

    std::vector<double> MoveItInterface::computeIK(const std::vector<double> &seed_positions,
                                                   const Eigen::Vector3d &target_position,
                                                   const std::array<double, 4> &target_orientation,
                                                   double timeout, int attempts,
                                                   const std::string &reference_frame)
    {
        // Convert Eigen types to geometry_msgs::Pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = target_position.x();
        target_pose.position.y = target_position.y();
        target_pose.position.z = target_position.z();
        target_pose.orientation.x = target_orientation[0];
        target_pose.orientation.y = target_orientation[1];
        target_pose.orientation.z = target_orientation[2];
        target_pose.orientation.w = target_orientation[3];
        
        return computeIK(seed_positions, target_pose, timeout, attempts, reference_frame);
    }

    bool MoveItInterface::getEndEffectorPose(const std::vector<double> &joint_positions,
                                             Eigen::Isometry3d &ee_pose_out,
                                             const std::string &reference_frame) const
    {
        if (!isMoveGroupValid(false))
        {
            RCLCPP_WARN(node_->get_logger(), "MoveGroup not initialized");
            return false;
        }

        auto current_state = move_group_->getCurrentState();
        auto jmg = current_state->getJointModelGroup(group_name_);

        if (!jmg)
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint model group not found");
            return false;
        }

        current_state->setJointGroupPositions(jmg, joint_positions);
        current_state->update();
        
        const Eigen::Isometry3d &pose_world = current_state->getGlobalLinkTransform(move_group_->getEndEffectorLink());
        
        // Determine the reference frame to use
        std::string ref_frame = reference_frame.empty() ? pose_reference_frame_ : reference_frame;
        
        // If a specific reference frame is provided, transform the pose
        if (!ref_frame.empty() && current_state->knowsFrameTransform(ref_frame))
        {
            const Eigen::Isometry3d T_world_ref = current_state->getGlobalLinkTransform(ref_frame);
            ee_pose_out = T_world_ref.inverse() * pose_world;
        }
        else
        {
            ee_pose_out = pose_world;
        }

        return true;
    }

    bool MoveItInterface::getEndEffectorPose(const std::vector<double> &joint_positions,
                                             geometry_msgs::msg::Pose &ee_pose_out,
                                             const std::string &reference_frame) const
    {
        Eigen::Isometry3d pose_eigen;
        if (!getEndEffectorPose(joint_positions, pose_eigen, reference_frame))
        {
            return false;
        }

        ee_pose_out.position.x = pose_eigen.translation().x();
        ee_pose_out.position.y = pose_eigen.translation().y();
        ee_pose_out.position.z = pose_eigen.translation().z();

        Eigen::Quaterniond quat(pose_eigen.rotation());
        ee_pose_out.orientation.x = quat.x();
        ee_pose_out.orientation.y = quat.y();
        ee_pose_out.orientation.z = quat.z();
        ee_pose_out.orientation.w = quat.w();

        return true;
    }

    bool MoveItInterface::validateIKSolution(const std::vector<double> &joint_angles,
                                             const Eigen::Isometry3d &target_pose,
                                             double &pos_error_out,
                                             double &angular_error_out,
                                             double pos_threshold,
                                             double angular_threshold) const
    {
        Eigen::Isometry3d computed_pose;
        if (!getEndEffectorPose(joint_angles, computed_pose))
        {
            return false;
        }

        // Position error
        Eigen::Vector3d pos_error = computed_pose.translation() - target_pose.translation();
        pos_error_out = pos_error.norm();

        // Angular error (rotation difference)
        Eigen::Quaterniond q1(computed_pose.rotation());
        Eigen::Quaterniond q2(target_pose.rotation());
        angular_error_out = q1.angularDistance(q2);

        return (pos_error_out < pos_threshold && angular_error_out < angular_threshold);
    }

    bool MoveItInterface::getEEToCameraTransform(const std::string &camera_link,
                                                 Eigen::Isometry3d &T_ee_cam_out) const
    {
        if (!isPSMValid(false))
            return false;

        const std::string ee_link = getEndEffectorLink();
        if (ee_link.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "End effector link is empty. Did MoveGroupInterface initialize correctly?");
            return false;
        }

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &state = scene->getCurrentState();

        if (!state.knowsFrameTransform(ee_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown EE frame: %s", ee_link.c_str());
            return false;
        }
        if (!state.knowsFrameTransform(camera_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown camera frame: %s", camera_link.c_str());
            return false;
        }

        // Global transforms
        const Eigen::Isometry3d T_world_ee = state.getGlobalLinkTransform(ee_link);
        const Eigen::Isometry3d T_world_cam = state.getGlobalLinkTransform(camera_link);

        // Relative EE->camera
        T_ee_cam_out = T_world_ee.inverse() * T_world_cam;
        return true;
    }

    bool MoveItInterface::cameraPoseToEEPose(const geometry_msgs::msg::Pose &cam_pose,
                                             const std::string &camera_link,
                                             geometry_msgs::msg::Pose &ee_pose_out,
                                             const std::string &reference_frame) const
    {
        Eigen::Isometry3d T_ee_cam;
        if (!getEEToCameraTransform(camera_link, T_ee_cam))
        {
            return false;
        }

        Eigen::Isometry3d T_ref_cam;
        tf2::fromMsg(cam_pose, T_ref_cam);
        
        // Determine the reference frame to use
        std::string ref_frame = reference_frame.empty() ? pose_reference_frame_ : reference_frame;
        
        // If a specific reference frame is provided, we need to transform to world frame first
        if (!ref_frame.empty() && isPSMValid(false))
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
            const moveit::core::RobotState &state = scene->getCurrentState();
            
            if (state.knowsFrameTransform(ref_frame))
            {
                const Eigen::Isometry3d T_world_ref = state.getGlobalLinkTransform(ref_frame);
                const Eigen::Isometry3d T_world_cam = T_world_ref * T_ref_cam;
                const Eigen::Isometry3d T_world_ee = T_world_cam * T_ee_cam.inverse();
                
                // Transform back to reference frame
                const Eigen::Isometry3d T_ref_ee = T_world_ref.inverse() * T_world_ee;
                ee_pose_out = tf2::toMsg(T_ref_ee);
                return true;
            }
        }
        
        // No frame transform needed, cam_pose is already in world frame
        const Eigen::Isometry3d T_world_ee = T_ref_cam * T_ee_cam.inverse();
        ee_pose_out = tf2::toMsg(T_world_ee);
        return true;
    }

    bool MoveItInterface::cameraPoseToEEPose(const Eigen::Vector3d &cam_position,
                                             const std::array<double, 4> &cam_orientation,
                                             const std::string &camera_link,
                                             Eigen::Vector3d &ee_position_out,
                                             std::array<double, 4> &ee_orientation_out,
                                             const std::string &reference_frame) const
    {
        // Convert Eigen types to geometry_msgs::Pose
        geometry_msgs::msg::Pose cam_pose;
        cam_pose.position.x = cam_position.x();
        cam_pose.position.y = cam_position.y();
        cam_pose.position.z = cam_position.z();
        cam_pose.orientation.x = cam_orientation[0];
        cam_pose.orientation.y = cam_orientation[1];
        cam_pose.orientation.z = cam_orientation[2];
        cam_pose.orientation.w = cam_orientation[3];
        
        geometry_msgs::msg::Pose ee_pose;
        if (!cameraPoseToEEPose(cam_pose, camera_link, ee_pose, reference_frame))
        {
            return false;
        }
        
        // Convert back to Eigen types
        ee_position_out.x() = ee_pose.position.x;
        ee_position_out.y() = ee_pose.position.y;
        ee_position_out.z() = ee_pose.position.z;
        ee_orientation_out[0] = ee_pose.orientation.x;
        ee_orientation_out[1] = ee_pose.orientation.y;
        ee_orientation_out[2] = ee_pose.orientation.z;
        ee_orientation_out[3] = ee_pose.orientation.w;
        
        return true;
    }

    // IK configuration
    double MoveItInterface::getIKTimeout() const
    {
        return ik_timeout_;
    }

    void MoveItInterface::setIKTimeout(double timeout)
    {
        ik_timeout_ = timeout;
    }

    // Planning configuration
    std::string MoveItInterface::getPlanningPipelineId() const
    {
        if (!isMoveGroupValid(false))
            return planning_pipeline_id_;
        return move_group_->getPlanningPipelineId();
    }

    void MoveItInterface::setPlanningPipelineId(const std::string pipeline_id)
    {
        planning_pipeline_id_ = pipeline_id;
        if (isMoveGroupValid(false))
            move_group_->setPlanningPipelineId(pipeline_id);
    }

    std::string MoveItInterface::getPlannerId() const
    {
        if (!isMoveGroupValid(false))
            return planner_id_;
        return move_group_->getPlannerId();
    }

    void MoveItInterface::setPlannerId(const std::string planner_id)
    {
        planner_id_ = planner_id;
        if (isMoveGroupValid(false)) {
            move_group_->setPlannerId(planner_id);
            RCLCPP_DEBUG(node_->get_logger(), "Set planner to: %s", planner_id.c_str());
        }
    }

    double MoveItInterface::getPlanningTime() const
    {
        if (!isMoveGroupValid(false))
            return planning_time_;
        return move_group_->getPlanningTime();
    }

    void MoveItInterface::setPlanningTime(double seconds)
    {
        planning_time_ = seconds;
        if (isMoveGroupValid(false)) {
            move_group_->setPlanningTime(seconds);
            RCLCPP_DEBUG(node_->get_logger(), "Set planning time to: %.2f seconds", seconds);
        }
    }

    int MoveItInterface::getNumPlanningAttempts() const
    {
        return num_planning_attempts_;
    }

    void MoveItInterface::setNumPlanningAttempts(int num_attempts)
    {
        num_planning_attempts_ = num_attempts;
        if (isMoveGroupValid(false)) {
            move_group_->setNumPlanningAttempts(num_attempts);
            RCLCPP_DEBUG(node_->get_logger(), "Set number of planning attempts to: %d", num_attempts);
        }
    }

    double MoveItInterface::getMaxVelocityScalingFactor() const
    {
        return max_velocity_scaling_factor_;
    }

    void MoveItInterface::setMaxVelocityScalingFactor(double scale)
    {
        if (scale <= 0.0 || scale > 1.0) {
            RCLCPP_WARN(node_->get_logger(), "Velocity scaling factor must be in (0.0, 1.0]. Clamping to valid range.");
            scale = std::clamp(scale, 0.01, 1.0);
        }
        
        max_velocity_scaling_factor_ = scale;
        if (isMoveGroupValid(false)) {
            move_group_->setMaxVelocityScalingFactor(scale);
            RCLCPP_DEBUG(node_->get_logger(), "Set max velocity scaling factor to: %.2f", scale);
        }
    }

    double MoveItInterface::getMaxAccelerationScalingFactor() const 
    {
        return max_acceleration_scaling_factor_;
    }

    void MoveItInterface::setMaxAccelerationScalingFactor(double scale)
    {
        if (scale <= 0.0 || scale > 1.0) {
            RCLCPP_WARN(node_->get_logger(), "Acceleration scaling factor must be in (0.0, 1.0]. Clamping to valid range.");
            scale = std::clamp(scale, 0.01, 1.0);
        }
        
        max_acceleration_scaling_factor_ = scale;
        if (isMoveGroupValid(false)) {
            move_group_->setMaxAccelerationScalingFactor(scale);
            RCLCPP_DEBUG(node_->get_logger(), "Set max acceleration scaling factor to: %.2f", scale);
        }
    }

    bool MoveItInterface::getCurrentEndEffectorPose(geometry_msgs::msg::Pose &pose_out)
    {
        // Use PSM-based getLinkPose — reads the cached planning scene state and does
        // not wait for a fresh joint_states timestamp (avoids post-execution race condition).
        const std::string ee_link = getEndEffectorLink();
        if (ee_link.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "getCurrentEndEffectorPose: end effector link is empty");
            return false;
        }
        return getLinkPose(ee_link, pose_out);
    }

    void MoveItInterface::setOrientationConstraints(
        const std::string& link_name,
        const geometry_msgs::msg::Quaternion& target_orientation,
        double tolerance_roll,
        double tolerance_pitch,
        double tolerance_yaw)
    {
        if (!isMoveGroupValid())
            return;

        // MoveIt expects tolerances in [0, pi]. Using pi means "no constraint" on that axis.
        auto sanitize_tol = [](double t) -> double {
            if (!std::isfinite(t) || t <= 0.0) return M_PI;      // unconstrained
            return std::min(t, M_PI);
        };

        const double tol_x = sanitize_tol(tolerance_roll);
        const double tol_y = sanitize_tol(tolerance_pitch);
        const double tol_z = sanitize_tol(tolerance_yaw);

        // Normalize quaternion (important for constraint projection).
        Eigen::Quaterniond q(target_orientation.w,
                            target_orientation.x,
                            target_orientation.y,
                            target_orientation.z);
        if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
            q.norm() < 1e-9)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "setOrientationConstraints: invalid quaternion for link '%s' — skipping",
                        link_name.c_str());
            return;
        }
        q.normalize();

        geometry_msgs::msg::Quaternion q_msg;
        q_msg.w = q.w(); q_msg.x = q.x(); q_msg.y = q.y(); q_msg.z = q.z();

        // IMPORTANT: Use planning frame for constraints unless you *explicitly* know another frame is correct.
        const std::string frame_id = move_group_->getPlanningFrame();

        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = link_name;
        ocm.header.frame_id = frame_id;
        ocm.orientation = q_msg;
        ocm.absolute_x_axis_tolerance = tol_x;
        ocm.absolute_y_axis_tolerance = tol_y;
        ocm.absolute_z_axis_tolerance = tol_z;
        ocm.weight = 1.0;

        // Merge with existing constraints, but REPLACE any existing orientation constraints for this link.
        moveit_msgs::msg::Constraints constraints = move_group_->getPathConstraints();

        constraints.orientation_constraints.erase(
            std::remove_if(constraints.orientation_constraints.begin(),
                        constraints.orientation_constraints.end(),
                        [&](const auto& c){ return c.link_name == link_name; }),
            constraints.orientation_constraints.end());

        // If all axes are effectively unconstrained, don't add anything.
        const bool unconstrained =
            (tol_x >= M_PI - 1e-9) && (tol_y >= M_PI - 1e-9) && (tol_z >= M_PI - 1e-9);

        if (!unconstrained)
            constraints.orientation_constraints.push_back(ocm);

        move_group_->setPathConstraints(constraints);

        RCLCPP_DEBUG(node_->get_logger(),
                    "Orientation constraints for '%s' in frame '%s' tol=[x:%.3f y:%.3f z:%.3f]",
                    link_name.c_str(), frame_id.c_str(), tol_x, tol_y, tol_z);
    }

    void MoveItInterface::setPositionConstraints(
        const std::string& link_name,
        double x_min, double x_max,
        double y_min, double y_max,
        double z_min, double z_max)
    {
        if (!isMoveGroupValid())
            return;

        auto is_unconstrained = [](double v) { return !std::isfinite(v); };

        // If every bound is unconstrained, remove any existing position constraint for this link and return.
        const bool all_uncon =
            is_unconstrained(x_min) && is_unconstrained(x_max) &&
            is_unconstrained(y_min) && is_unconstrained(y_max) &&
            is_unconstrained(z_min) && is_unconstrained(z_max);

        moveit_msgs::msg::Constraints constraints = move_group_->getPathConstraints();

        // Always REPLACE existing position constraints for this link.
        constraints.position_constraints.erase(
            std::remove_if(constraints.position_constraints.begin(),
                        constraints.position_constraints.end(),
                        [&](const auto& c){ return c.link_name == link_name; }),
            constraints.position_constraints.end());

        if (all_uncon)
        {
            move_group_->setPathConstraints(constraints);
            RCLCPP_DEBUG(node_->get_logger(),
                        "setPositionConstraints: all unconstrained — removed any existing constraint for '%s'",
                        link_name.c_str());
            return;
        }

        // Replace unconstrained sides with large bounds.
        // NOTE: This is still a box constraint (PositionConstraint only supports regions),
        // but we avoid pathological huge numbers.
        constexpr double WS = 5.0;  // meters
        if (is_unconstrained(x_min)) x_min = -WS;
        if (is_unconstrained(x_max)) x_max =  WS;
        if (is_unconstrained(y_min)) y_min = -WS;
        if (is_unconstrained(y_max)) y_max =  WS;
        if (is_unconstrained(z_min)) z_min = -WS;
        if (is_unconstrained(z_max)) z_max =  WS;

        // Ensure ordering and non-degenerate sizes.
        auto fix_bounds = [](double& mn, double& mx, const char* axis, rclcpp::Logger logger){
            if (!std::isfinite(mn) || !std::isfinite(mx))
                return;
            if (mn > mx) std::swap(mn, mx);
            if (std::abs(mx - mn) < 1e-6)
            {
                // Expand a tiny bit to avoid zero-size primitive.
                const double c = 0.5 * (mn + mx);
                mn = c - 5e-4;
                mx = c + 5e-4;
                RCLCPP_WARN(logger, "setPositionConstraints: %s bounds were degenerate; expanded slightly", axis);
            }
        };

        fix_bounds(x_min, x_max, "x", node_->get_logger());
        fix_bounds(y_min, y_max, "y", node_->get_logger());
        fix_bounds(z_min, z_max, "z", node_->get_logger());

        const double cx = 0.5 * (x_min + x_max);
        const double cy = 0.5 * (y_min + y_max);
        const double cz = 0.5 * (z_min + z_max);
        const double sx = std::max(x_max - x_min, 1e-4);
        const double sy = std::max(y_max - y_min, 1e-4);
        const double sz = std::max(z_max - z_min, 1e-4);

        moveit_msgs::msg::PositionConstraint pcm;
        pcm.header.frame_id = move_group_->getPlanningFrame();  // match the planning frame
        pcm.link_name = link_name;
        pcm.weight = 1.0;

        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions.resize(3);
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sx;
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sy;
        box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sz;

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = cx;
        box_pose.position.y = cy;
        box_pose.position.z = cz;
        box_pose.orientation.w = 1.0;

        pcm.constraint_region.primitives.clear();
        pcm.constraint_region.primitive_poses.clear();
        pcm.constraint_region.primitives.push_back(box);
        pcm.constraint_region.primitive_poses.push_back(box_pose);

        constraints.position_constraints.push_back(pcm);
        move_group_->setPathConstraints(constraints);

        RCLCPP_DEBUG(node_->get_logger(),
                    "Position constraint for '%s' frame '%s': x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f]",
                    link_name.c_str(), pcm.header.frame_id.c_str(),
                    x_min, x_max, y_min, y_max, z_min, z_max);
    }

    void MoveItInterface::clearPathConstraints()
    {
        if (!isMoveGroupValid())
            return;
        
        move_group_->clearPathConstraints();
        RCLCPP_DEBUG(node_->get_logger(), "Cleared all path constraints");
    }


    /**
     * @brief Compute L2 distance between two joint configurations
     */
    double MoveItInterface::jointDeltaL2(const std::vector<double>& a, const std::vector<double>& b)
    {
        if (a.size() != b.size()) return std::numeric_limits<double>::infinity();
        double s2 = 0.0;
        for (size_t i=0; i<a.size(); ++i) {
            const double d = a[i] - b[i];
            s2 += d*d;
        }
        return std::sqrt(s2);
    }

    // Helper functions for advanced planning
    double MoveItInterface::computeJointSpacePathLength(const trajectory_msgs::msg::JointTrajectory &trajectory)
    {
        if (trajectory.points.size() < 2)
            return std::numeric_limits<double>::infinity();

        double total_length = 0.0;
        for (size_t i = 1; i < trajectory.points.size(); ++i)
        {
            const auto &prev_positions = trajectory.points[i - 1].positions;
            const auto &curr_positions = trajectory.points[i].positions;
            
            if (prev_positions.size() != curr_positions.size())
                return std::numeric_limits<double>::infinity();

            double segment_length_sq = 0.0;
            for (size_t j = 0; j < prev_positions.size(); ++j)
            {
                double delta = curr_positions[j] - prev_positions[j];
                segment_length_sq += delta * delta;
            }
            total_length += std::sqrt(segment_length_sq);
        }
        return total_length;
    }

    bool MoveItInterface::hasStrictlyIncreasingTime(const trajectory_msgs::msg::JointTrajectory &trajectory)
    {
        if (trajectory.points.size() < 2)
            return true;

        rclcpp::Duration prev_time(0, 0);
        for (size_t i = 0; i < trajectory.points.size(); ++i)
        {
            rclcpp::Duration curr_time(trajectory.points[i].time_from_start);
            if (i > 0 && curr_time <= prev_time)
                return false;
            prev_time = curr_time;
        }
        return true;
    }

    void MoveItInterface::ensureStrictlyIncreasingTime(trajectory_msgs::msg::JointTrajectory &trajectory, double dt_sec)
    {
        if (trajectory.points.empty())
            return;

        // If already strictly increasing, do nothing
        if (hasStrictlyIncreasingTime(trajectory))
            return;

        // Force strictly increasing timestamps
        for (size_t i = 0; i < trajectory.points.size(); ++i)
        {
            trajectory.points[i].time_from_start = rclcpp::Duration::from_seconds(dt_sec * static_cast<double>(i));
        }
    }

    std::vector<double> MoveItInterface::jitterJointSeed(const std::vector<double> &seed, double sigma, std::mt19937 &rng)
    {
        std::normal_distribution<double> noise_dist(0.0, sigma);
        std::vector<double> jittered_seed = seed;
        for (double &value : jittered_seed)
            value += noise_dist(rng);
        return jittered_seed;
    }

    bool MoveItInterface::planToTargetPoseWithRetries(const geometry_msgs::msg::Pose &target_ee_pose,
                                                       moveit::planning_interface::MoveGroupInterface::Plan &best_plan_out,
                                                       int num_ik_seeds,
                                                       int plans_per_seed,
                                                       double ik_timeout,
                                                       int ik_attempts,
                                                       const std::vector<std::vector<double>> &hint_seeds)
    {
        // Get current joint state as baseline seed
        std::vector<double> current_joint_state;
        if (!getCurrentJointAngles(current_joint_state))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get current joint state for IK seed");
            return false;
        }

        // Random number generator for seed jittering (fixed seed for reproducibility)
        std::mt19937 rng(42u);

        // Build ordered seed list: current state → caller hints → jittered random
        std::vector<std::vector<double>> seeds;
        seeds.push_back(current_joint_state);
        for (const auto &hint : hint_seeds)
        {
            if (hint.size() == current_joint_state.size())
                seeds.push_back(hint);
            else
                RCLCPP_WARN(node_->get_logger(), "Hint seed size %zu != expected %zu, skipping.",
                            hint.size(), current_joint_state.size());
        }
        while (static_cast<int>(seeds.size()) < num_ik_seeds)
            seeds.push_back(jitterJointSeed(current_joint_state, 0.4, rng));

        // Track best plan found
        bool found_valid_plan = false;
        double best_cost = std::numeric_limits<double>::infinity();
        const double dt_sec = 0.02; // Time step for trajectory timestamps

        RCLCPP_DEBUG(node_->get_logger(),
                    "Attempting to find best plan with %zu seeds (%zu hints) and %d plans per seed",
                    seeds.size(), hint_seeds.size(), plans_per_seed);

        // Try seeds in order
        for (int seed_idx = 0; seed_idx < static_cast<int>(seeds.size()); ++seed_idx)
        {
            const std::vector<double> &seed = seeds[seed_idx];

            // Compute IK with this seed
            auto ik_solution = computeIK(seed, target_ee_pose, ik_timeout, ik_attempts);
            if (ik_solution.empty())
                continue;

            // Quick collision check
            if (!isStateValid(ik_solution))
                continue;

            // Try multiple planning attempts with this IK solution
            for (int plan_idx = 0; plan_idx < plans_per_seed; ++plan_idx)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool planning_success = planToJointGoal(ik_solution, plan);
                
                if (!planning_success || plan.trajectory_.joint_trajectory.points.empty())
                    continue;

                // Ensure trajectory has strictly increasing timestamps (required by controller)
                ensureStrictlyIncreasingTime(plan.trajectory_.joint_trajectory, dt_sec);

                // Compute cost as joint-space path length
                double path_cost = computeJointSpacePathLength(plan.trajectory_.joint_trajectory);

                // Keep best plan
                if (!found_valid_plan || path_cost < best_cost)
                {
                    best_cost = path_cost;
                    best_plan_out = plan;
                    found_valid_plan = true;
                    
                    const char* seed_type = (seed_idx == 0) ? "current"
                        : (seed_idx <= static_cast<int>(hint_seeds.size()) ? "hint" : "jittered");
                    RCLCPP_DEBUG(node_->get_logger(),
                                "Found better plan (cost=%.3f) with %s seed %d, attempt %d",
                                path_cost, seed_type, seed_idx, plan_idx);
                }
            }

            // Early exit if we found a very good solution
            if (found_valid_plan && best_cost < 2.0)
            {
                RCLCPP_DEBUG(node_->get_logger(), "Found sufficiently good plan early, stopping search");
                break;
            }
        }

        if (found_valid_plan)
        {
            RCLCPP_INFO(node_->get_logger(), 
                       "Selected best plan with joint-space path length: %.3f", best_cost);
            return true;
        }

        RCLCPP_WARN(node_->get_logger(),
                   "No valid plan found after trying %zu seeds (%zu hints + jittered)",
                   seeds.size(), hint_seeds.size());
        return false;
    }
    // Frame configuration
    std::string MoveItInterface::getPoseReferenceFrame() const
    {
        if (!isMoveGroupValid(false))
            return pose_reference_frame_;
        return move_group_->getPoseReferenceFrame();
    }

    void MoveItInterface::setPoseReferenceFrame(const std::string &frame)
    {
        pose_reference_frame_ = frame;
        if (isMoveGroupValid(false)) {
            move_group_->setPoseReferenceFrame(frame);
            RCLCPP_DEBUG(node_->get_logger(), "Set pose reference frame to: %s", frame.c_str());
        }
    }

    std::string MoveItInterface::getEndEffectorLink() const
    {
        if (!isMoveGroupValid(false))
            return end_effector_link_;
        return move_group_->getEndEffectorLink();
    }

    void MoveItInterface::setEndEffectorLink(const std::string &link)
    {
        end_effector_link_ = link;
        if (isMoveGroupValid(false)) {
            move_group_->setEndEffectorLink(link);
            RCLCPP_DEBUG(node_->get_logger(), "Set end effector link to: %s", link.c_str());
        }
    }

    // Robot structure queries
    std::vector<std::string> MoveItInterface::getManipulatorLinks() const
    {
        std::vector<std::string> links;

        moveit::core::JointModelGroupConstPtr jmg;
        if (!const_cast<MoveItInterface *>(this)->getJointModelGroup(jmg))
            return links;

        return jmg->getLinkModelNames();
    }

    std::vector<std::string> MoveItInterface::getManipulatorJointNames() const
    {
        std::vector<std::string> joints;

        moveit::core::JointModelGroupConstPtr jmg;
        if (!const_cast<MoveItInterface *>(this)->getJointModelGroup(jmg))
            return joints;

        return jmg->getActiveJointModelNames();
    }

    std::vector<std::string> MoveItInterface::getAllRobotLinkNames() const
    {
        std::vector<std::string> links;

        if (!isPSMValid(false))
            return links;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotModelConstPtr &robot_model = scene->getRobotModel();

        const std::vector<const moveit::core::LinkModel *> &link_models = robot_model->getLinkModels();
        for (const auto *link : link_models)
        {
            links.push_back(link->getName());
        }

        return links;
    }

    std::vector<std::string> MoveItInterface::getAllRobotJointNames() const
    {
        std::vector<std::string> joints;

        if (!isPSMValid(false))
            return joints;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotModelConstPtr &robot_model = scene->getRobotModel();

        const std::vector<const moveit::core::JointModel *> &joint_models = robot_model->getJointModels();
        for (const auto *joint : joint_models)
        {
            joints.push_back(joint->getName());
        }

        return joints;
    }

    // Transform queries
    bool MoveItInterface::getTransformBetweenLinks(const std::string &from_link,
                                                   const std::string &to_link,
                                                   Eigen::Isometry3d &transform) const
    {
        if (!isPSMValid(false))
            return false;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &state = scene->getCurrentState();

        if (!state.knowsFrameTransform(from_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown frame: %s", from_link.c_str());
            return false;
        }

        if (!state.knowsFrameTransform(to_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown frame: %s", to_link.c_str());
            return false;
        }

        const Eigen::Isometry3d &from_transform = state.getGlobalLinkTransform(from_link);
        const Eigen::Isometry3d &to_transform = state.getGlobalLinkTransform(to_link);

        transform = from_transform.inverse() * to_transform;
        return true;
    }

    bool MoveItInterface::getLinkPose(const std::string &link_name,
                                      geometry_msgs::msg::Pose &pose,
                                      const std::string &reference_frame) const
    {
        if (!isPSMValid(false))
            return false;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &state = scene->getCurrentState();

        if (!state.knowsFrameTransform(link_name))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown link: %s", link_name.c_str());
            return false;
        }

        const Eigen::Isometry3d &pose_world = state.getGlobalLinkTransform(link_name);
        
        // Determine the reference frame to use
        std::string ref_frame = reference_frame.empty() ? pose_reference_frame_ : reference_frame;
        
        // If a specific reference frame is provided, transform the pose
        if (!ref_frame.empty() && state.knowsFrameTransform(ref_frame))
        {
            const Eigen::Isometry3d T_world_ref = state.getGlobalLinkTransform(ref_frame);
            const Eigen::Isometry3d pose_in_ref = T_world_ref.inverse() * pose_world;
            pose = tf2::toMsg(pose_in_ref);
        }
        else
        {
            pose = tf2::toMsg(pose_world);
        }

        return true;
    }

    bool MoveItInterface::getLinkPose(const std::string &link_name,
                                      Eigen::Vector3d &position,
                                      std::array<double, 4> &orientation,
                                      const std::string &reference_frame) const
    {
        geometry_msgs::msg::Pose pose;
        if (!getLinkPose(link_name, pose, reference_frame))
        {
            return false;
        }
        
        // Convert to Eigen types
        position.x() = pose.position.x;
        position.y() = pose.position.y;
        position.z() = pose.position.z;
        orientation[0] = pose.orientation.x;
        orientation[1] = pose.orientation.y;
        orientation[2] = pose.orientation.z;
        orientation[3] = pose.orientation.w;
        
        return true;
    }

} // namespace erwinia_os_nbv_planner
