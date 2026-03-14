# Instructions to run autonomous navigation

source install/setup.bash

1. Run the interface with amiga. This will give you /odom and amiga will be drivable through /cmd_vel commands.
```
ros2 run amiga_control amiga_control
```

2. Run nav sensors: IMU, GPS and Velodyne
```
ros2 launch erwinia_launch nav_sensors.launch.py 
```

3. Convert IMU to enu coords
```
ros2 run vectornav imu_ned2enu_node
```

4. Convert gps rtk baseline msg to enu coordinates
```
ros2 run mpc_amiga convert_from_swiftnav.py 
```

5. Run robot localization
```
ros2 launch robot_localization dual_ekf_amiga.launch.py
```

6. To collect waypoints run this command
```
python3 src/erwinia_navigation/control/MPC_Amiga/scripts/collect_goals.py
```
Remove the first 4 lines of the barn_to_field_waypoints.txt. 
Copy the last point to stopping points and stopping points copied.

7. To run the controller run
```
ros2 launch mpc_amiga mpc_amiga.launch.py fresh_start:=true
```
This command will show in the terminal the current yaw of the robot and the yaw of the path to track. If the robot is aligned to the path, it is expeced to have similar yaw. If they are different, there is an issue.

Remember to always check the path and odometry before engaging `auto control` in the amiga pendant.


8. Run MPPI 

sudo apt install ros-humble-nav2-controller
colcon build --packages-select amiga_local_planner && source install/setup.bash
