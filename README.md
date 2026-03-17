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
By default the input device is keyboard. If you want to use the Xbox controller to trigger BT pass 'xbox' as an argument:
```bash
BT_INTERACTIVE=1 ros2 run erwinia_bt bt_executor  --ros-args -p interactive_input_device:=xbox
```
Make sure the joy node is running, you can run it
```bash
ros2 run joy joy_node
```
### Terminal 2: Navigation teleop node

```bash
ros2 launch amiga_control control.launch.py
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
# erwinia_os
Repository to host all the code for the Farm Robotics Challenge 2026.

## Package Overview

- **`erwinia_os_bringup`** - Main orchestration package that launches the complete robot stack including Gazebo (optional), ros2_control, and RViz visualization. Use this for typical system startup.

- **`erwinia_os_control`** - ros2_control configuration package with dynamic YAML generation for the diff_drive_controller. Provides launch files for starting the control system with Gazebo simulation, fake hardware, or real hardware interfaces.

- **`erwinia_os_gz`** - Gazebo simulation assets including world files and launch configurations for testing in simulation environments.

- **`erwinia_os_description`** - Robot description package containing URDF/Xacro files, STL meshes, and RViz configurations for the Amiga platform. Includes modular macros for the base, wheels, camera, and LiDAR sensors with namespace/prefix support.

## Contributing workflow (git)

```bash
# Start from latest main
cd ~/ros2_ws/src/erwinia_os
git checkout main     # Moves to main branch
git pull origin main  # Updates main branch

# Create a feature branch
git checkout -b <username>/<feature-to-be>

# Make changes, then stage and commit
git status         # Shows you what files have been updated/added/removed
git add <files>    # Can use git add . to add all of the updates/additions/deletions
git commit -m "Brief description of change"

# Push your branch and open a PR
git push -u origin <username>/<feature-to-be>
git push           # Should work after setting the upstream ('-u origin <username>/<feature-to-be>')
# In GitHub/GitLab, open a merge request targeting main and fill in summary + testing

# After PR is merged and successful, delete the old feature branch
# This will be deleted remotely once merged by one of the repo managers

# Please feel free to safely delete it from your local machine AFTER the merge request is accepted
git checkout main
git pull origin main      # Update local main with merged changes
git branch -d <username>/<feature-to-be>  # Delete local branch

# IN THE MEANTIME, you may begin a new feature branch for the next feature intented to be added


# Useful git checks and navigation
git branch               # Show current branch (* indicates active)
git switch <name>        # Switch to another branch (or: git checkout <name>)
git restore <file>       # Drop local changes to a file (or: git checkout -- <file>)
git pull origin main     # Update the current branch (be sure to do this regularly)
```

## Quick Start

### Launch Full System in Simulation
```bash
ros2 launch erwinia_os_bringup bringup.launch.py use_gazebo:=true use_fake_hardware:=false use_sim_time:=true
```

### Launch with Fake Hardware (Mock Components)
```bash
ros2 launch erwinia_os_bringup bringup.launch.py use_gazebo:=false use_fake_hardware:=true use_sim_time:=true
```

### Launch with Real Hardware (TODO)
```bash
ros2 launch erwinia_os_bringup bringup.launch.py use_gazebo:=false use_fake_hardware:=false use_sim_time:=false
```

### View Robot Description in RViz
```bash
ros2 launch erwinia_os_description view_robot.launch.py
```

### Control the Robot
```bash
# Publish velocity commands (or control through gazebo sim gui)
ros2 topic pub /platform_velocity_controller/cmd_vel_unstamped \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# Monitor odometry
ros2 topic echo /odom

# Monitor joint states
ros2 topic echo /joint_states
```

## Common Commands

### Build and Source
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select erwinia_os_description erwinia_os_control erwinia_os_bringup erwinia_os_gz
source install/setup.bash
```

### Check Active Controllers
```bash
ros2 control list_controllers
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
# Or in RViz: Add TF display
```

### Inspect URDF
```bash
# Process xacro to URDF
xacro $(ros2 pkg prefix erwinia_os_description)/share/erwinia_os_description/urdf/erwinia_os.urdf.xacro

# Check for errors
check_urdf <path_to_urdf>
```

## Getting set up (ROS 2 Humble on Ubuntu 22)

1. Install ROS 2 Humble by following the official guide: https://docs.ros.org/en/humble/Installation.html (use the instructions for your OS).
2. Create a workspace and clone this repo under `src`:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Kantor-Lab/erwinia_os.git
   ```
3. Import dependency repositories using vcstool:
    ```bash
    # Install vcs if needed
    sudo apt install python3-vcstool

    cd ~/ros2_ws
    vcs import src/ < src/erwinia_os/dependencies.repos
    ```
4. Install ROS2 package dependencies via rosdep:
    ```bash
    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths src/ --ignore-src -r -y
    ```
5. To update the git modules inside of the multi camera rig package:
    ```bash
    git -C src/multi_camera_rig_v3 submodule update --init --recursive
    ```
6. Sync and update the xarm packages:
    ```bash
    git -C src/xarm_ros2 submodule sync --recursive && \
    git -C src/xarm_ros2 submodule update --init --recursive
    ```
7. Build and source ROS2 workspace:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

