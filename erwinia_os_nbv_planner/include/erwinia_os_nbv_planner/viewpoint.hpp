#pragma once

#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <optional>

namespace erwinia_os_nbv_planner
{

/**
 * @brief Represents a candidate viewpoint for NBV planning (moveit planning frame!)
 */
struct Viewpoint
{
    Eigen::Vector3d position;                              ///< 3D camera position
    std::array<double, 4> orientation;                     ///< Camera orientation (quaternion [x,y,z,w])
    std::optional<Eigen::Vector3d> target;                 ///< 3D point the camera looks at
    double information_gain = 0.0;                         ///< Information gain score
    double cost = 0.0;                                     ///< Movement cost
    double utility = 0.0;                                  ///< Final utility (IG - α*cost)
    std::optional<std::vector<double>> joint_angles;       ///< Robot joint angles for this viewpoint
    
    Viewpoint() : orientation{0.0, 0.0, 0.0, 1.0} {}
    
    Viewpoint(const Eigen::Vector3d& pos, const std::array<double, 4>& orient)
        : position(pos), orientation(orient) {}
    
    /**
     * @brief Comparison operator for max-heap ordering
     * Returns true if this viewpoint has LOWER utility than other
     * This creates a max-heap where highest utility is at top
     */
    bool operator<(const Viewpoint& other) const
    {
        return utility < other.utility;
    }
};

} // namespace erwinia_os_nbv_planner
