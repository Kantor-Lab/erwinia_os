# MPC in ROS2 humble

This MPC package for global navigation uses [ros2 humble](https://docs.ros.org/en/humble/Installation.html) and Ubuntu 22.04.

## Create the workspace
```
mkdir -p amiga_ws/src
cd amiga_ws/src
git clone -b ros2-humble https://github.com/Kantor-Lab/MPC_Amiga.git
```
## Setting up the ACADO toolkit
If you are using Python 3.10, you don't need to generate it by yourself. The [acado.cpython-310-x86_64-linux-gnu.so](https://github.com/Kantor-Lab/MPC_Amiga/blob/ros2-humble/src/) will automatically been
installed in the ROS2 workspace.

If you are using another Python version, generate the acado.so file based on the tutorial in the [main](https://github.com/Kantor-Lab/MPC_Amiga) branch. And put the generated acado.so in the MPC_Amiga/src/ folder. 
Change the file name in the [CMakelist.txt](https://github.com/Kantor-Lab/MPC_Amiga/blob/ros2-humble/CMakeLists.txt#L69).

```
cd ~/amiga_ws
colcon build --symlink-install
```
## Global Waypoints
Put the global waypoints in the [barn_field_waypoints.txt](https://github.com/Kantor-Lab/MPC_Amiga/blob/ros2-humble/gps_coordinates/barn_field_waypoints.txt) in the same format. 

Put the waypoint of the final goal in [stopping_points.txt](https://github.com/Kantor-Lab/MPC_Amiga/blob/ros2-humble/gps_coordinates/stopping_points.txt) twice, same for the [stopping_points_copied.txt](https://github.com/Kantor-Lab/MPC_Amiga/blob/ros2-humble/gps_coordinates/stopping_points_copied.txt)

## Launch the package
```
ros2 launch mpc_amiga Mpc_amiga.launch.py fresh_start:=true
```

https://github.com/user-attachments/assets/deda2889-3183-4399-825f-4c4380eda359



https://github.com/user-attachments/assets/75192abe-35aa-4ee4-9a56-e2040d31bcf5

