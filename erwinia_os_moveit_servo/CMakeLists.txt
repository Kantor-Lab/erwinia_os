cmake_minimum_required(VERSION 3.8)
project(erwinia_os_moveit_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(xbox_to_servo_pub SHARED
  src/xbox_to_servo_pub.cpp
)

ament_target_dependencies(xbox_to_servo_pub
  rclcpp rclcpp_components sensor_msgs geometry_msgs control_msgs std_srvs
)

rclcpp_components_register_nodes(xbox_to_servo_pub
  "erwinia_os_moveit_servo::XboxToServoPub"
)

install(TARGETS xbox_to_servo_pub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(xbox_to_servo_pub)
ament_export_dependencies(rclcpp rclcpp_components)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  erwinia_os_moveit_servo/generate_servo_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Python module
ament_python_install_package(${PROJECT_NAME})

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
