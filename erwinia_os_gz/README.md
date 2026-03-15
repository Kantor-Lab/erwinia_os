# Erwinia OS Gazebo Package

This package provides Gazebo simulation worlds and models for the Erwinia OS system.

## Launch Files

### gazebo.launch.py

General-purpose launch file for starting Gazebo with various worlds.

**Usage:**

```bash
# Launch with apple orchard world (default)
ros2 launch erwinia_os_gz gazebo.launch.py

# Launch with empty world
ros2 launch erwinia_os_gz gazebo.launch.py world:=empty

# Launch with custom world from this package
ros2 launch erwinia_os_gz gazebo.launch.py world:=my_custom_world.sdf

# Launch with absolute path to world file
ros2 launch erwinia_os_gz gazebo.launch.py world:=/path/to/custom.sdf
```

**Arguments:**

- `world` (default: `apple_orchard`) - World to load:
  - `empty` - Default Gazebo empty world
  - `apple_orchard` - Custom apple orchard world from this package
  - Custom world name (e.g., `my_world.sdf`) - Looks in this package's worlds/ directory
  - Absolute path - Full path to a .sdf world file

**Features:**

- Automatically sets up Gazebo resource paths for custom models and worlds
- Uses `ros_gz_sim` for proper plugin loading (required for gz_ros2_control)
- Starts Gazebo in running mode (`-r` flag)

## Worlds

### apple_orchard.sdf

A simulated apple orchard environment with:
- Textured ground plane
- Apple tree rows with collision
- Suitable for agricultural robotics testing

Located in: `worlds/apple_orchard.sdf`

## Models

### apple_orchard

Custom model for the orchard environment including:
- Ground mesh with dirt texture
- Tree bed meshes with apple tree atlas texture
- Proper collision geometry

Located in: `models/apple_orchard/`

**Note:** Large mesh files (.obj) are stored using Git LFS.

## Usage with Bringup Package

The bringup package uses this launch file to start Gazebo:

```bash
# Full system with apple orchard
ros2 launch erwinia_os_bringup bringup.launch.py

# Full system with empty world
ros2 launch erwinia_os_bringup bringup.launch.py world:=empty

# Full system with custom world
ros2 launch erwinia_os_bringup bringup.launch.py world:=/path/to/world.sdf
```

## Adding New Worlds

To add a new world to this package:

1. Create your world SDF file in `worlds/` directory
2. Add any custom models to `models/` directory
3. Launch with: `ros2 launch erwinia_os_gz gazebo.launch.py world:=your_world.sdf`

## Resource Paths

The launch file automatically configures:
- `IGN_GAZEBO_RESOURCE_PATH` - For Ignition Fortress
- `GZ_SIM_RESOURCE_PATH` - For newer Gazebo versions

These include:
- This package's `worlds/` and `models/` directories
- All ROS 2 package share directories
- Any existing resource paths from your environment

## Dependencies

- `ros_gz_sim` - ROS 2 / Gazebo integration
- Git LFS - For large mesh files (install with `git lfs install`)

## Files

```
erwinia_os_gz/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE
├── launch/
│   └── gazebo.launch.py          # General Gazebo launcher
├── worlds/
│   └── apple_orchard.sdf         # Apple orchard world
└── models/
    └── apple_orchard/            # Orchard model
        ├── model.config
        ├── model.sdf
        ├── materials/
        │   ├── dirt.jpg
        │   └── apple_tree_atlas.jpg
        └── meshes/
            ├── ground.obj        # (Git LFS)
            └── bed1.obj          # (Git LFS)
```
