# NBV Volumetric Planner Summary

## Overview
Complete implementation of a Next-Best-View (NBV) planner for autonomous robotic exploration using octomap-based frontier detection and information gain maximization.

## Architecture

### Core Components
- **OctoMapInterface**: Thread-safe octomap access, frontier detection, k-means clustering
- **MoveItInterface**: Motion planning, IK/FK, collision checking
- **ManipulationWorkspace**: Robot reachability workspace learning and querying
- **Viewpoints**: Viewpoint generation (spherical cap, planar grid, hemisphere) and information gain computation
- **NBVVisualizer**: RViz visualization for frontiers, clusters, viewpoints

### Main Algorithm (8-Step Loop)
1. **Find Frontiers**: Detect free voxels with unknown neighbors in octomap
2. **Cluster Frontiers**: K-means clustering to group frontier regions
3. **Generate Viewpoints**: Hemispherical sampling around each cluster center
4. **Filter by Workspace**: Keep only reachable viewpoints
5. **Compute Information Gain**: Ray-casting to estimate observable unknown space
6. **Compute Utility**: `utility = IG - α × cost` (cost = Euclidean distance)
7. **Select Best Viewpoint**: Priority queue with IK/collision validation
8. **Execute Motion**: Plan and move to selected viewpoint

### Termination Conditions
- Maximum iterations reached
- Information gain below threshold
- No valid reachable viewpoints found

## Key Files

### Source Files
- `src/nbv_volumetric_planner_demo.cpp` - Main NBV planning loop
- `src/viewpoints.cpp` - Viewpoint generation and IG computation
- `src/octomap_interface.cpp` - Octomap operations and frontier detection
- `src/moveit_interface.cpp` - Motion planning abstractions
- `src/manipulation_workspace.cpp` - Workspace learning
- `src/nbv_visualizer.cpp` - RViz visualization

### Headers
- `include/erwinia_os_nbv_planner/*.hpp` - Corresponding headers
- `include/erwinia_os_nbv_planner/geometry_utils.hpp` - Quaternion/rotation utilities

## Launch

There are two launch modes:

### 1. Standalone Demo Node (`nbv_demo.launch.py`)

Launches the full NBV pipeline as a self-contained node, including the Firefly cameras, occupancy mapping, and the NBV planner. The node runs autonomously and terminates when a stopping condition is met.

```bash
ros2 launch erwinia_os_nbv_planner nbv_demo.launch.py \
  planner_type:=baseline \ # or: volumetric, semantic
  learn_workspace:=false \
  visualize_nbv:=false \
  n_runs:=1 \
  capture_type:=triggered \
  camera_max_range:=0.9 \
  map_frame:=amiga/base_footprint
```

### 2. Action Server (`nbv_action_server.launch.py`)

Launches only the NBV action server node. The Firefly cameras and occupancy mapping must already be running before launching this. The server exposes a `/run_nbv` action and waits for goals.

```bash
# Prerequisites (must be running first):
# - Firefly camera pipeline
# - Octomap server (erwinia_os_occupancy_map)

ros2 launch erwinia_os_nbv_planner nbv_action_server.launch.py \
  params_file:=<path_to_params.yaml>
```

Trigger a run once the server is up:

```bash
ros2 action send_goal /run_nbv erwinia_os_nbv_planner/action/RunNBV \
  "{planner_type: baseline}" --feedback
```

**Goal fields:**
- `planner_type` — planner to use: `baseline`, `volumetric`, `semantic`, or `paper`

**Feedback** (published after each viewpoint):
- `current_viewpoint` — index of the viewpoint just visited
- `cluster_count` — current number of detected semantic clusters
- `bbox_coverage` — bounding-box coverage fraction so far (0–1)
- `state` — short status string describing the current step

**Result** (on completion):
- `success` — whether the run completed successfully
- `message` — human-readable summary
- `total_viewpoints` — total viewpoints visited during the run
- `final_cluster_count` — semantic clusters at end of run
- `final_bbox_coverage` — final bounding-box coverage (0–1)

### Key Parameters
- `max_iterations`: Maximum NBV iterations (default: 50)
- `min_information_gain`: IG termination threshold (default: 0.1)
- `alpha_cost_weight`: Cost weight in utility function (default: 0.01)
- `num_viewpoints_per_frontier`: Candidates per cluster (default: 10)
- `camera_horizontal_fov`: Camera FOV horizontal (default: 1.5708 rad / 90°)
- `camera_vertical_fov`: Camera FOV vertical (default: 1.0472 rad / 60°)
- `camera_max_range`: Max sensing range (default: 5.0 m)
- `num_camera_rays`: Rays for IG computation (default: 1000)
- `octomap_topic`: Octomap topic (default: /octomap_binary)

### Workspace Parameters
- `learn_workspace`: Learn new workspace or load existing (default: true)
- `num_samples`: Workspace learning samples (default: 1000000)
- `manipulation_workspace_file`: Auto-generated timestamped file

## Build

```bash
colcon build --packages-select erwinia_os_nbv_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Features
✅ Frontier-based exploration  
✅ Information gain maximization  
✅ Robot workspace constraints  
✅ IK/collision validation  
✅ K-means frontier clustering  
✅ Multiple viewpoint generation strategies  
✅ Ray-casting IG computation  
✅ Priority queue viewpoint selection  
✅ RViz visualization integration  
✅ Configurable camera model  
✅ Automatic workspace learning/loading  

## Dependencies
- ROS2 Humble
- MoveIt2
- Octomap
- Eigen3
- geometry_msgs, sensor_msgs, visualization_msgs
