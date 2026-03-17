cmake_minimum_required(VERSION 3.5)
project(amiga_local_planner)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/path_relay.py
  scripts/obstacle_detector.py
  scripts/cmd_vel_switcher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
