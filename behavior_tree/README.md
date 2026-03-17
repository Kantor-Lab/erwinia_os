# Behavior Tree operation

The main BT executor is launched with `erwinia_bt`. The required supporting nodes depend on whether you are running in interactive or full autonomous mode.

## Prerequisites

- ROS 2 installed and sourced
- Workspace built with `colcon`

Example:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Interactive mode (semi-autonomous)

In interactive mode, the joystick is used for both:

- teleoperation of the robot
- success/failure input for BT nodes

`NavigateToPose` does not use the MPC navigation action server in this mode. Instead, `bt_executor.launch.py` starts the Amiga teleop stack and sets `BT_INTERACTIVE=1`.

Open 3 terminals.

Terminal 1: BT executor + teleop

```bash
ros2 launch erwinia_bt bt_executor.launch.py interactive:=true interactive_input_device:=xbox
```

Default Xbox behavior:

- `A` marks the current BT action as success
- `X` marks the current BT action as failure
- `Y` sets the current BT action as auto
- hold `RB` as the teleop enable button
- use the left joystick to drive the robot


Terminal 2: detector server

```bash
ros2 launch erwinia_os_nbv_planner nbv_action_server.launch.py
```

Terminal 3: mark server

```bash
ros2 launch erwinia_spray spray_marker.launch.py
```

## Full autonomous mode

In full autonomous mode, the BT expects the MPC action server providing `go_to_tree` to already be running. Launch the MPC action server before starting the BT executor.

Open 4 terminals.

Terminal 1: MPC action server

```bash
ros2 launch mpc_amiga mpc_amiga_server.launch.py
```
Terminal 2: detector server

```bash
ros2 launch erwinia_os_nbv_planner nbv_action_server.launch.py
```
Terminal 3: mark server

```bash
ros2 launch erwinia_spray spray_marker.launch.py
```

Terminal 4: BT executor

```bash
ros2 launch erwinia_bt bt_executor.launch.py interactive:=false
```





### Required in both modes

Regardless of whether the BT is interactive or non-interactive:

- the detector server must be launched
- the real mark server must be launched

## Optional: Visualize BT with Groot

You can visualize the behavior tree at runtime using Groot.

Install Groot:

- https://www.behaviortree.dev/groot/
- https://github.com/BehaviorTree/Groot

Build from source:

```bash
git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git
cd Groot
cmake -S . -B build
cmake --build build
```

Run Groot:

```bash
cd build
./Groot
```

Then connect Groot to the BT publisher IP/port from your running executor to see the live visualization.