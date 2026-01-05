# erwinia_os
Repository to host all the code for the Farm Robotics Challenge 2026.

Currently contains the initial ROS 2 package scaffolding:
- `erwinia_os_description` - robot description assets (URDF/Xacro, meshes, RViz configs)
- `erwinia_os_bringup` - launch files to compose and start the full system
- `erwinia_os_control` - ros2_control configuration and launch scaffolding
- `erwinia_os_gz` - Gazebo worlds, models, and launch scaffolding for simulation

Populate the empty `launch/`, `config/`, `urdf/`, and related folders to flesh out the stack.
