cmake_minimum_required(VERSION 3.8)
project(erwinia_os_nbv_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(erwinia_os_occupancy_map REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ompl REQUIRED)

# Generate custom action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunNBV.action"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Define the MoveIt interface library
add_library(moveit_interface SHARED
  src/moveit_interface.cpp
)
target_include_directories(moveit_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
)
ament_target_dependencies(moveit_interface
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)
target_link_libraries(moveit_interface ${OMPL_LIBRARIES})

# Define the OctoMap interface library
add_library(octomap_interface SHARED
  src/octomap_interface.cpp
)
target_include_directories(octomap_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(octomap_interface ${OCTOMAP_LIBRARIES})
ament_target_dependencies(octomap_interface
  rclcpp
  octomap_msgs
  visualization_msgs
  erwinia_os_occupancy_map
)

add_library(ground_truth_evaluator SHARED
  src/ground_truth_evaluator.cpp
)
target_include_directories(ground_truth_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ground_truth_evaluator
  octomap_interface
  yaml-cpp
)
ament_target_dependencies(ground_truth_evaluator
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Define the NBV visualizer library
add_library(nbv_visualizer SHARED
  src/nbv_visualizer.cpp
)
target_include_directories(nbv_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(nbv_visualizer ${OCTOMAP_LIBRARIES})
ament_target_dependencies(nbv_visualizer
  rclcpp
  visualization_msgs
  geometry_msgs
  std_msgs
)

# Define the Manipulation Workspace library
add_library(manipulation_workspace SHARED
  src/manipulation_workspace.cpp
)
target_include_directories(manipulation_workspace PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(manipulation_workspace 
  moveit_interface 
  nbv_visualizer
)
ament_target_dependencies(manipulation_workspace
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)

# Define the Viewpoint Generation library
add_library(viewpoint_generation SHARED
  src/viewpoint_generation.cpp
)
target_include_directories(viewpoint_generation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(viewpoint_generation 
  moveit_interface
  octomap_interface
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(viewpoint_generation
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)

# # Define the MoveIt interface demo executable
# add_executable(moveit_interface_demo src/moveit_interface_demo.cpp)
# target_link_libraries(moveit_interface_demo moveit_interface)
# ament_target_dependencies(moveit_interface_demo
#   rclcpp
# )

# # Define the OctoMap interface demo executable
# add_executable(octomap_interface_demo src/octomap_interface_demo.cpp)
# target_link_libraries(octomap_interface_demo octomap_interface nbv_visualizer)
# ament_target_dependencies(octomap_interface_demo
#   rclcpp
#   visualization_msgs
#   std_msgs
#   geometry_msgs
# )

# Define the NBV Semantic Planner demo executable
add_executable(nbv_semantic_planner_demo src/nbv_semantic_planner_demo.cpp)
target_link_libraries(nbv_semantic_planner_demo 
  moveit_interface 
  manipulation_workspace
  octomap_interface
  ground_truth_evaluator
  nbv_visualizer
  viewpoint_generation
)
ament_target_dependencies(nbv_semantic_planner_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)

# Define the NBV Volumetric Planner demo executable
add_executable(nbv_volumetric_planner_demo src/nbv_volumetric_planner_demo.cpp)
target_link_libraries(nbv_volumetric_planner_demo 
  moveit_interface 
  manipulation_workspace
  octomap_interface
  ground_truth_evaluator
  nbv_visualizer
  viewpoint_generation
)
ament_target_dependencies(nbv_volumetric_planner_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)

# Define the NBV Baseline demo executable
add_executable(nbv_baseline_demo src/nbv_baseline_demo.cpp)
target_link_libraries(nbv_baseline_demo 
  moveit_interface 
  manipulation_workspace
  octomap_interface
  ground_truth_evaluator
  nbv_visualizer
  viewpoint_generation
)
ament_target_dependencies(nbv_baseline_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)

add_executable(nbv_replay src/nbv_replay.cpp)
target_link_libraries(nbv_replay 
  moveit_interface 
  manipulation_workspace
  octomap_interface
  ground_truth_evaluator
  nbv_visualizer
  viewpoint_generation
)
ament_target_dependencies(nbv_replay
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
  sensor_msgs
)

# Define the NBV Baseline action server executable
add_executable(nbv_action_server src/nbv_action_server.cpp)
target_link_libraries(nbv_action_server
  moveit_interface
  manipulation_workspace
  octomap_interface
  ground_truth_evaluator
  nbv_visualizer
  viewpoint_generation
  "${cpp_typesupport_target}"
)
ament_target_dependencies(nbv_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
)

# Install rules
install(TARGETS 
  moveit_interface 
  octomap_interface
  ground_truth_evaluator
  nbv_visualizer
  manipulation_workspace
  viewpoint_generation
#   moveit_interface_demo
#   octomap_interface_demo
  nbv_semantic_planner_demo
  nbv_volumetric_planner_demo
  nbv_baseline_demo
  nbv_replay
  nbv_action_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch metrics config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
