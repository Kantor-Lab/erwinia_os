cmake_minimum_required(VERSION 3.5)
project(mpc_amiga)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add the C++ and C source files for the acado library
add_library(acado_diff SHARED
  src/acado_diff/acado.c
  src/acado_diff/acado_auxiliary_functions.c
  src/acado_diff/acado_integrator.c
  src/acado_diff/acado_solver.c
  src/acado_diff/acado_qpoases_interface.cpp
  src/acado_diff/qpoases/SRC/Bounds.cpp
  src/acado_diff/qpoases/SRC/Constraints.cpp
  src/acado_diff/qpoases/SRC/CyclingManager.cpp
  src/acado_diff/qpoases/SRC/Indexlist.cpp
  src/acado_diff/qpoases/SRC/MessageHandling.cpp
  src/acado_diff/qpoases/SRC/QProblem.cpp
  src/acado_diff/qpoases/SRC/QProblemB.cpp
  src/acado_diff/qpoases/SRC/SubjectTo.cpp
  src/acado_diff/qpoases/SRC/Utils.cpp
  src/acado_diff/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavToTree.action"
)

# Specify include directories
target_include_directories(acado_diff
  PRIVATE
    include
    include/MPC_amiga/acado
    src/acado_diff
    src/acado_diff/qpoases
    src/acado_diff/qpoases/INCLUDE
    src/acado_diff/qpoases/SRC # Include path for qpoases headers
)
set(Python3_INCLUDE_DIRS "/usr/include/python3.10")
target_include_directories(acado_diff PRIVATE ${Python3_INCLUDE_DIRS})

# Link necessary libraries to your executable
target_link_libraries(
  acado_diff  # Link the acado_diff library
)

# Add dependencies for the executable


# Install the executables and libraries
install(TARGETS acado_diff
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES src/acado.cpython-310-x86_64-linux-gnu.so
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages/
)

# Install Python scripts
install(PROGRAMS
    scripts/mpc_warthog_v4.py
    scripts/mpc_warthog_v4_action.py
    scripts/collect_goals.py
    scripts/convert_from_swiftnav.py
    DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY scripts/test.py
#   DESTINATION lib/${PROJECT_NAME}/
#   FILES_MATCHING PATTERN "*.py"
# )
install(
  DIRECTORY common/
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages/common
)
install(
  DIRECTORY gps_coordinates/
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages/gps_coordinates
)
# Install header files from the acado_diff and qpoases folders
install(DIRECTORY src/acado_diff/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

# Ensure the package is built and installed
ament_package()
