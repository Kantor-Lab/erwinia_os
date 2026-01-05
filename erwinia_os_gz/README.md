# Erwinia OS Gazebo

Scaffolding for Gazebo simulation assets for the Erwinia platform. Add SDF worlds to `worlds/`, custom models to `models/`, and launch files under `launch/` to start Gazebo with the desired environment.

## Quick start

Launch Gazebo with a world:

```bash
ros2 launch erwinia_os_gz gazebo.launch.py world:=fb_tree
ros2 launch erwinia_os_gz gazebo.launch.py world:=empty
```

Once the bringup package is built out, this world can be optionally brought up in the bringup subpackage (if the desired bringup is a simulation) by including something like this:

```python
if use_gazebo_val:
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            "world": world,
        }.items(),
    )
    launch_actions.append(gazebo_launch)
```
