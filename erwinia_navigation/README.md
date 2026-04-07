# Instructions to run autonomous navigation

 ros2 launch erwinia_os_description description.launch.py

1. Pre-requisites — run in separate terminals before anything else:
```
ros2 run rmw_zenoh_cpp rmw_zenohd
str2str -in ntrip://fyandun@andrew.cmu.edu:@rtk2go.com:2101/cmuairlab01 -out serial://ttyUSB1:115200:8:n:1
```


2. Run the interface with amiga. This will give you /odom and amiga will be drivable through /cmd_vel commands. For this, first setup the can connection:

```
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
candump can0
```

Then launch the node:
```
ros2 run amiga_control amiga_control
```

3. Run nav sensors: IMU, GPS and Velodyne
```
ros2 launch erwinia_os_bringup nav_sensors.launch.py
```

4. Convert IMU to enu coords
```
ros2 run vectornav imu_ned2enu_node
```

5. Convert gps rtk baseline msg to enu coordinates
```
ros2 run mpc_amiga convert_from_swiftnav.py
```

6. Run robot localization
```
ros2 launch robot_localization dual_ekf_amiga.launch.py
```

7. To collect waypoints run this command
```
python3 src/erwinia_os/erwinia_navigation/control/MPC_Amiga/scripts/collect_goals.py
```
Remove the first 4 lines of the barn_to_field_waypoints.txt.
Copy the last point to stopping points and stopping points copied.

8. To run the controller run

**If you want obstacle avoidance, skip this step and go directly to step 9.**

```
ros2 launch mpc_amiga mpc_amiga.launch.py fresh_start:=true
```
This command will show in the terminal the current yaw of the robot and the yaw of the path to track. If the robot is aligned to the path, it is expeced to have similar yaw. If they are different, there is an issue.

Remember to always check the path and odometry before engaging `auto control` in the amiga pendant.


9. Run MPC + MPPI Combo (Obstacle Avoidance)

> `mpc_amiga` is launched internally — no need to run it separately.
> For architecture details and parameter tuning, see [amiga_local_planner/README.md](control/amiga_local_planner/README.md).

> **Progress:** Only tested `planner:=mppi dry_run:=false` — robot spins at the end of the path. `planner:=combo` has not been tested on the real robot yet.

> Known issue: spinning at end of path with `planner:=mppi`. Likely due to `GoalAngleCritic` or `GoalCritic` weights being too high in `mppi.yaml` — see [amiga_local_planner/README.md](control/amiga_local_planner/README.md) for tuning.

**Step 1 — Start in debug mode** (skip to **Step 3 Final run** if already verified):
```
ros2 launch amiga_local_planner mpc_mppi_combo.launch.py planner:=mpc dry_run:=true
```

**Arguments:**

| Argument | Options | Behavior |
|----------|---------|----------|
| `planner` | `combo` | Dynamically switches between MPC and MPPI — uses MPPI when obstacles are detected, MPC otherwise |
| | `mpc` | MPC only; no obstacle avoidance (mirrors `mpc_amiga`) |
| | `mppi` | MPPI only; no obstacle avoidance |
| `dry_run` | `true` | Costmap active, `cmd_vel` held at zero — for visualization and pre-run checks |
| | `false` | Live mode — velocity commands sent to the robot |

**Step 2 — Test with live commands** (single planner, no switching):
```
ros2 launch amiga_local_planner mpc_mppi_combo.launch.py planner:=mpc dry_run:=false
```
```
ros2 launch amiga_local_planner mpc_mppi_combo.launch.py planner:=mppi dry_run:=false
```

**Step 3 — Final run** (full combo mode):
```
ros2 launch amiga_local_planner mpc_mppi_combo.launch.py planner:=combo dry_run:=false
```
