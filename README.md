# Erwinia Behavior Tree (Main BT + Dummy Action Nodes)

This workspace hosts the **main behavior tree** for the Erwinia project.

At this stage, action nodes are implemented as **placeholder/dummy ROS 2 servers**. These will be replaced by real implementations as they become available.

## What this includes

- `erwinia_bt`: behavior tree executor
- `erwinia_nav_server`: dummy navigation action server
- `erwinia_detector_server`: dummy detector action server
- `erwinia_mark_server`: dummy marker action server

## Prerequisites

- ROS 2 installed and sourced
- Workspace built with `colcon`

Example:

```bash
cd ~/erwinia_ws
colcon build
source install/setup.bash
```

## Run the system

Open **4 terminals** in total.

### Terminal 1: Behavior Tree executor

```bash
BT_INTERACTIVE=1 ros2 run erwinia_bt bt_executor
```

### Terminal 2: Navigation dummy node

```bash
ros2 run erwinia_nav_server navigation_server
```

### Terminal 3: Detector dummy node

```bash
ros2 run erwinia_detector_server erwinia_detector_server
```

### Terminal 4: Mark dummy node

```bash
ros2 run erwinia_mark_server erwinia_mark_server
```

## Optional: Visualize BT with Groot

You can visualize the behavior tree at runtime using Groot.

### Install Groot

Official project page:
- https://www.behaviortree.dev/groot/

Source repository (build-from-source instructions):
- https://github.com/BehaviorTree/Groot

Linux build steps (from the official Groot repository):

```bash
git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git
cd Groot
cmake -S . -B build
cmake --build build
```

### Run Groot

From the Groot repository root:

```bash
cd build
./Groot
```

Then connect Groot to the BT publisher IP/port from your running executor to see the live visualization.
