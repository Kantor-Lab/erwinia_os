# erwinia_os
Repository to host all the code for the Farm Robotics Challenge 2026.

## Package Overview

- **`erwinia_os_bringup`** - Main orchestration package that launches the complete robot stack including Gazebo (optional), ros2_control, and RViz visualization. Use this for typical system startup.

- **`erwinia_os_control`** - ros2_control configuration package with dynamic YAML generation for the diff_drive_controller. Provides launch files for starting the control system with Gazebo simulation, fake hardware, or real hardware interfaces.

- **`erwinia_os_gz`** - Gazebo simulation assets including world files and launch configurations for testing in simulation environments.

- **`erwinia_os_description`** - Robot description package containing URDF/Xacro files, STL meshes, and RViz configurations for the Amiga platform. Includes modular macros for the base, wheels, camera, and LiDAR sensors with namespace/prefix support.

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

## Getting set up (ROS 2 Humble)

1. Install ROS 2 Humble by following the official guide: https://docs.ros.org/en/humble/Installation.html (use the instructions for your OS).
2. Create a workspace and clone this repo under `src`:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/Kantor-Lab/erwinia_os.git
   ```
3. Source your ROS 2 environment, then build:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

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

