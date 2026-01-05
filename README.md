# erwinia_os
Repository to host all the code for the Farm Robotics Challenge 2026.

Currently contains the initial ROS 2 package scaffolding:
- `erwinia_os_description` - robot description assets (URDF/Xacro, meshes, RViz configs)
- `erwinia_os_bringup` - launch files to compose and start the full system
- `erwinia_os_control` - ros2_control configuration and launch scaffolding
- `erwinia_os_gz` - Gazebo worlds, models, and launch scaffolding for simulation

Populate the empty `launch/`, `config/`, `urdf/`, and related folders to flesh out the stack.

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
git checkout main
git pull origin main

# Create a feature branch
git checkout -b <username>/<feature-to-be>

# Make changes, then stage and commit
git status         # Shows you what files have been updated/added/removed
git add <files>    # Can use git add . to add all of the updates/additions/deletions
git commit -m "Brief description of change"

# Push your branch and open a PR
git push -u origin <username>/<feature-to-be>
# In GitHub/GitLab, open a merge request targeting main and fill in summary + testing


# Useful git checks and navigation
git branch               # Show current branch (* indicates active)
git switch <name>        # Switch to another branch (or: git checkout <name>)
git restore <file>       # Drop local changes to a file (or: git checkout -- <file>)
git pull origin main     # Update the current branch (be sure to do this regularly)
```
