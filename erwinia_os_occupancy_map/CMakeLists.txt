cmake_minimum_required(VERSION 3.8)
project(erwinia_os_occupancy_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomOctomap.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    octomap_msgs
)

# Get the typesupport target for use in same package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
)

# ============================================================================
# Core Library: Thread-safe Octomap wrapper
# ============================================================================
add_library(${PROJECT_NAME}_core
  src/occupancy_map_tree.cpp
  src/semantic_occupancy_map_tree.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  octomap_msgs
)

# ============================================================================
# Monitor Library: Map management and updating
# ============================================================================
add_library(${PROJECT_NAME}_monitor
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  src/pointcloud_updater.cpp
  src/semantic_pointcloud_updater.cpp
)
target_include_directories(${PROJECT_NAME}_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_monitor
  ${PROJECT_NAME}_core
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_monitor
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================================
# Visualization Library: RViz markers for debugging
# ============================================================================
add_library(${PROJECT_NAME}_visualization
  src/occupancy_map_visualizer.cpp
)
target_include_directories(${PROJECT_NAME}_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_visualization
  ${PROJECT_NAME}_core
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_visualization
  rclcpp
  visualization_msgs
  std_msgs
)

# ============================================================================
# Executables
# ============================================================================

# MoveIt integration node
add_executable(octomap_server
  src/octomap_server.cpp
)
target_link_libraries(octomap_server
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_monitor
  ${PROJECT_NAME}_visualization
  ${OCTOMAP_LIBRARIES}
  ${cpp_typesupport_target}
)
ament_target_dependencies(octomap_server
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
  octomap_msgs
  moveit_msgs
  visualization_msgs
)

# ============================================================================
# Octomap Save/Load Utilities
# ============================================================================
# add_executable(octomap_saver
#   src/octomap_saver.cpp
# )
# target_link_libraries(octomap_saver
#   ${OCTOMAP_LIBRARIES}
# )
# ament_target_dependencies(octomap_saver
#   rclcpp
#   octomap_msgs
# )

# add_executable(octomap_loader
#   src/octomap_loader.cpp
# )
# target_link_libraries(octomap_loader
#   ${OCTOMAP_LIBRARIES}
# )
# ament_target_dependencies(octomap_loader
#   rclcpp
#   octomap_msgs
#   visualization_msgs
#   geometry_msgs
#   std_msgs
# )

# ============================================================================
# Install
# ============================================================================

# Install libraries
install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_monitor
  ${PROJECT_NAME}_visualization
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
  octomap_server
#   octomap_saver
#   octomap_loader
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# # Install data directory for runtime octomap storage
# install(DIRECTORY
#   data
#   DESTINATION share/${PROJECT_NAME}/
# )

# ============================================================================
# Export
# ============================================================================
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  octomap
  octomap_msgs
  moveit_msgs
  rosidl_default_runtime
)
ament_export_include_directories(include)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
