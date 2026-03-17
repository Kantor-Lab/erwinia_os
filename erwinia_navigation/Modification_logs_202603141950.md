# Modification Log — MPC Amiga Controller

**Date:** 2026-03-14
**Author:** Appleseed Labs
**Files Modified:**
- `MPC_Amiga/scripts/mpc_warthog_v4.py`
- `MPC_Amiga/common/utils.py`

---

## mpc_warthog_v4.py

### 1. Hardcoded `sys.path` redirect

**Line 8**

**Before:**
```python
sys.path.append("/home/appleseed_labs/erwinia_os/install/mpc_amiga/lib/python3.10/dist-packages/")
```

**After:**
```python
sys.path.insert(0, "/home/appleseed_labs/erwinia_test_2_ws/install/mpc_amiga/lib/python3.10/dist-packages/")
```

**Reason:**
The script was loading `common/utils.py` from the `erwinia_os` workspace install directory instead of `erwinia_test_2_ws`. Using `sys.path.insert(0, ...)` ensures the correct workspace takes priority, so edits to source files in `erwinia_test_2_ws` are reflected after a rebuild.

---

### 2. `yaw_prev_` initialization fix

**Line 29 and Lines 179–183**

**Before:**
```python
yaw_prev_ = 0
...
yaw_in_range = utils.wrapTopm2Pi(euler_meas, yaw_prev_)
robot_state.set_meas(x_meas, y_meas, yaw_in_range, v_meas, w_meas)
yaw_prev_ = yaw_in_range
```

**After:**
```python
yaw_prev_ = None
...
if yaw_prev_ is None:
    yaw_prev_ = euler_meas
yaw_in_range = utils.wrapTopm2Pi(euler_meas, yaw_prev_)
robot_state.set_meas(x_meas, y_meas, yaw_in_range, v_meas, w_meas)
yaw_prev_ = yaw_in_range
```

**Reason:**
`wrapTopm2Pi` computes yaw continuity by comparing the current measurement to the previous value. When initialized to `0`, if the robot starts at any heading other than East (0 rad), the first callback sees a large `dyaw` (potentially `> π/2`) and applies an incorrect `2π` wrap. This offset then propagates permanently, causing reported robot yaw to be misaligned from path yaw by a multiple of `2π` even when the robot is physically aligned with the path.

Initializing `yaw_prev_` to `None` and seeding it from the first actual odometry reading ensures the unwrapping starts from the true initial heading with zero error.

---

## common/utils.py

### 1. `calc_nearest_index` — empty sequence guard

**Line 400**

**Before:**
```python
def calc_nearest_index(state, cx, cy, cyaw, pind):
    dx = [state.x - icx for icx in cx[pind:(pind + defs.N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + defs.N_IND_SEARCH)]]
```

**After:**
```python
def calc_nearest_index(state, cx, cy, cyaw, pind):
    pind = min(pind, len(cx) - 1)
    dx = [state.x - icx for icx in cx[pind:(pind + defs.N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + defs.N_IND_SEARCH)]]
```

**Reason:**
When `pind` exceeded the length of `cx` (e.g., near the end of the path), the slice `cx[pind:(pind + N_IND_SEARCH)]` returned an empty list, making `d` empty and causing:
```
ValueError: min() arg is an empty sequence
```
Clamping `pind` to `len(cx) - 1` ensures the search window always contains at least one point.

---

### 2. `get_pruning_points` — 1D array indexing fix

**Line 271**

**Before:**
```python
pruning_points = np.loadtxt(path_to_load, delimiter=',', dtype=float)
px = pruning_points[:,0].tolist()
py = pruning_points[:,1].tolist()
```

**After:**
```python
pruning_points = np.atleast_2d(np.loadtxt(path_to_load, delimiter=',', dtype=float))
px = pruning_points[:,0].tolist()
py = pruning_points[:,1].tolist()
```

**Reason:**
When the stopping points file contains only a single row, `np.loadtxt` returns a 1D array of shape `(N,)` instead of a 2D array of shape `(1, N)`. Column indexing with `[:,0]` then fails with:
```
IndexError: too many indices for array: array is 1-dimensional, but 2 were indexed
```
Wrapping with `np.atleast_2d()` ensures the array is always 2D regardless of the number of rows in the file.

---

---

## mpc_warthog_v4.py (continued)

### 3. `/aPath` — frame_id and timestamp fix

**Lines 274–281**

**Before:**
```python
my_path = Path()
my_path.header.frame_id = 'enu'
for x, y in zip(global_cx, global_cy):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    my_path.poses.append(pose)
```

**After:**
```python
my_path = Path()
my_path.header.frame_id = 'enu'
my_path.header.stamp = self.get_clock().now().to_msg()
for x, y in zip(global_cx, global_cy):
    pose = PoseStamped()
    pose.header.frame_id = 'enu'
    pose.header.stamp = my_path.header.stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    my_path.poses.append(pose)
```

**Reason:**
The path header had a zero timestamp (`sec: 0, nanosec: 0`) and individual poses had an empty `frame_id`. RViz2 ignores messages with zero timestamps and cannot render poses without a valid frame. Setting the stamp from the node clock and assigning `frame_id: enu` to each pose makes the path visible in RViz2 when the fixed frame is set to `enu`.

---

## amiga_local_planner config

### 1. `costmap.yaml` — global_frame set to `enu`

**Before:**
```yaml
global_frame: velodyne
```

**After:**
```yaml
global_frame: enu
```

**Reason:**
`velodyne` is a sensor body frame (child of `amiga/base_link`). Using it as `global_frame` would make the costmap static relative to the sensor instead of rolling with the robot in the world. `enu` is the root world frame published by the GPS/EKF localization stack (`enu → odom → amiga/base_footprint → velodyne`).

---

### 2. `mppi.yaml` — controller_frequency increased to match model_dt

**Before:**
```yaml
controller_frequency: 10.0
model_dt: 0.05
```

**After:**
```yaml
controller_frequency: 20.0
model_dt: 0.05
```

**Reason:**
The MPPI controller requires `controller_period ≤ model_dt`. With `controller_frequency: 10.0`, the period was 0.1s which exceeded `model_dt: 0.05s`, causing the controller_server to abort on configure with:
```
Controller period more than model dt, set it equal to model dt
```
Raising frequency to 20Hz (period = 0.05s) satisfies the constraint.

---

### 3. `costmap.yaml` — z_voxels capped at 16

**Before:**
```yaml
z_voxels: !!int 20
```

**After:**
```yaml
z_voxels: !!int 16
```

**Reason:**
The `nav2_costmap_2d::VoxelLayer` implementation supports a maximum of 16 z values. Setting 20 triggered:
```
Error, this implementation can only support up to 16 z values (20)
```

---

## Summary

| File | Issue | Fix |
|------|-------|-----|
| `mpc_warthog_v4.py` | Wrong workspace `sys.path` | Redirected to `erwinia_test_2_ws` |
| `mpc_warthog_v4.py` | `yaw_prev_ = 0` causing 2π offset on startup | Initialize from first odometry reading |
| `mpc_warthog_v4.py` | `/aPath` zero timestamp + empty pose frame_id | Set stamp from clock, set `frame_id: enu` on each pose |
| `utils.py` | `min()` on empty list in `calc_nearest_index` | Clamp `pind` to valid range |
| `utils.py` | 1D array column indexing in `get_pruning_points` | Wrap with `np.atleast_2d()` |
| `costmap.yaml` | `global_frame: velodyne` (sensor body frame) | Changed to `global_frame: enu` |
| `mppi.yaml` | Controller period > model_dt → SIGABRT on configure | Raised `controller_frequency` to 20Hz |
| `costmap.yaml` | `z_voxels: 20` exceeds VoxelLayer max of 16 | Capped to `z_voxels: 16` |
