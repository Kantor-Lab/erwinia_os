cmake_minimum_required(VERSION 3.5)
project(erwinia_bt_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(erwinia_os_nbv_planner REQUIRED)
find_package(erwinia_msgs REQUIRED)
find_package(mpc_amiga REQUIRED)

add_library(erwinia_bt_nodes SHARED src/erwinia_bt_nodes.cpp)
ament_target_dependencies(erwinia_bt_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  sensor_msgs
  erwinia_os_nbv_planner
  erwinia_msgs
  mpc_amiga)

target_compile_definitions(erwinia_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS erwinia_bt_nodes
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
