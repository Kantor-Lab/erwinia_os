#!/usr/bin/env python3
"""
Orchestrate NBV planner experiments with rosbag playback.

Sequence for each experiment:
1. Launch nbv_demo (in its own process group)
2. Wait for it to initialize
3. Play the rosbag
4. When the rosbag process ends, shut down nbv_demo (whole process group)
5. Wait for nodes to fully die, then move to the next experiment

KEY DETAIL: `ros2 launch` spawns child node processes. Killing only the
launch process orphans those nodes (they keep running and show up in
`ros2 topic list`). To avoid that, every subprocess is started in its own
session/process group via start_new_session=True, and we signal the ENTIRE
group with os.killpg(). We send SIGINT first (same as Ctrl+C, which lets
ros2 launch tear down its nodes cleanly), then escalate to SIGTERM/SIGKILL.
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

# ============================================================================
# CONFIGURATION - Edit these lists to change which trees/runs to process
# ============================================================================

# List of experiments to process:
#   (rosbag_path, run_name, gt_points_file, extra_params)
# extra_params is an optional dict of additional launch arguments that override
# or extend NBV_BASE_PARAMS for that specific run (e.g. {"decimation_factor": "2"}).
EXPERIMENT_CONFIGS: List[Tuple[str, str, str, dict]] = [
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_eval_recordings/recording_baseline_2/",
        "tree_12/baseline/",
        "metrics/penn_state/tree_12/gt_points.json",
        {"decimation_factor": "1.64"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_eval_recordings/recording_volumetric_2/",
        "tree_12/volumetric/",
        "metrics/penn_state/tree_12/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_eval_recordings/recording_semantic_2/",
        "tree_12/semantic/",
        "metrics/penn_state/tree_12/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_eval_recordings/recording_baseline_1/",
        "tree_11/baseline/",
        "metrics/penn_state/tree_11/gt_points.json",
        {"decimation_factor": "1.94"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_eval_recordings/recording_volumetric_1/",
        "tree_11/volumetric/",
        "metrics/penn_state/tree_11/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_eval_recordings/recording_semantic_1/",
        "tree_11/semantic/",
        "metrics/penn_state/tree_11/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_10/",
        "tree_10/baseline/",
        "metrics/penn_state/tree_10/gt_points.json",
        {"decimation_factor": "1"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_10/",
        "tree_10/volumetric/",
        "metrics/penn_state/tree_10/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_10/",
        "tree_10/semantic/",
        "metrics/penn_state/tree_10/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_9/",
        "tree_9/baseline/",
        "metrics/penn_state/tree_9/gt_points.json",
        {"decimation_factor": "1.14"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_9/",
        "tree_9/volumetric/",
        "metrics/penn_state/tree_9/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_9/",
        "tree_9/semantic/",
        "metrics/penn_state/tree_9/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_8/",
        "tree_8/baseline/",
        "metrics/penn_state/tree_8/gt_points.json",
        {"decimation_factor": "1.67"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_8/",
        "tree_8/volumetric/",
        "metrics/penn_state/tree_8/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_8/",
        "tree_8/semantic/",
        "metrics/penn_state/tree_8/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_7/",
        "tree_7/baseline/",
        "metrics/penn_state/tree_7/gt_points.json",
        {"decimation_factor": "1.4"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_7/",
        "tree_7/volumetric/",
        "metrics/penn_state/tree_7/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_7/",
        "tree_7/semantic/",
        "metrics/penn_state/tree_7/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_6/",
        "tree_6/baseline/",
        "metrics/penn_state/tree_6/gt_points.json",
        {"decimation_factor": "1"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_6/",
        "tree_6/volumetric/",
        "metrics/penn_state/tree_6/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_6/",
        "tree_6/semantic/",
        "metrics/penn_state/tree_6/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_5/",
        "tree_5/baseline/",
        "metrics/penn_state/tree_5/gt_points.json",
        {"decimation_factor": "1.04"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_5/",
        "tree_5/volumetric/",
        "metrics/penn_state/tree_5/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_5/",
        "tree_5/semantic/",
        "metrics/penn_state/tree_5/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_4/",
        "tree_4/baseline/",
        "metrics/penn_state/tree_4/gt_points.json",
        {"decimation_factor": "1.27"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_4/",
        "tree_4/volumetric/",
        "metrics/penn_state/tree_4/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_4/",
        "tree_4/semantic/",
        "metrics/penn_state/tree_4/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_3/",
        "tree_3/baseline/",
        "metrics/penn_state/tree_3/gt_points.json",
        {"decimation_factor": "1"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_3/",
        "tree_3/volumetric/",
        "metrics/penn_state/tree_3/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_3/",
        "tree_3/semantic/",
        "metrics/penn_state/tree_3/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_2/",
        "tree_2/baseline/",
        "metrics/penn_state/tree_2/gt_points.json",
        {"decimation_factor": "1.34"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_2/",
        "tree_2/volumetric/",
        "metrics/penn_state/tree_2/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_2/",
        "tree_2/semantic/",
        "metrics/penn_state/tree_2/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_baseline_1/",
        "tree_1/baseline/",
        "metrics/penn_state/tree_1/gt_points.json",
        {"decimation_factor": "1.64"},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_volumetric_1/",
        "tree_1/volumetric/",
        "metrics/penn_state/tree_1/gt_points.json",
        {},
    ),
    (
        "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2/robot_demo_recordings/day1/recording_semantic_1/",
        "tree_1/semantic/",
        "metrics/penn_state/tree_1/gt_points.json",
        {},
    ),
]

# ============================================================================
# ABLATION STUDIES — generated programmatically to avoid repetition.
#
# For every (baseline, semantic) pair across all trees we run 8 extra passes,
# each varying one parameter at two extremes (low / high) while keeping all
# other params at their NBV_BASE_PARAMS defaults.
#
# Naming: tree_N/<planner>_<param>_<level>/
#   e.g.  tree_5/baseline_res_low/   tree_5/semantic_conf_high/
# ============================================================================

_EVAL_BASE = "/media/hayden/T7/penn_state_3-26/fireblight_data_days1-2"

# (tree_id, baseline_bag, semantic_bag)
_ABLATION_TREES: List[Tuple[int, str, str]] = [
    (12, f"{_EVAL_BASE}/robot_eval_recordings/recording_baseline_2/",
         f"{_EVAL_BASE}/robot_eval_recordings/recording_semantic_2/"),
    (11, f"{_EVAL_BASE}/robot_eval_recordings/recording_baseline_1/",
         f"{_EVAL_BASE}/robot_eval_recordings/recording_semantic_1/"),
    (10, f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_10/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_10/"),
    (9,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_9/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_9/"),
    (8,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_8/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_8/"),
    (7,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_7/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_7/"),
    (6,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_6/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_6/"),
    (5,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_5/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_5/"),
    (4,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_4/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_4/"),
    (3,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_3/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_3/"),
    (2,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_2/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_2/"),
    (1,  f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_baseline_1/",
         f"{_EVAL_BASE}/robot_demo_recordings/day1/recording_semantic_1/"),
]

# (run_name_suffix, param_overrides)
# The suffix becomes part of the output directory name so results are
# self-documenting.  Baseline runs additionally carry decimation_factor=2.
_ABLATION_PASSES: List[Tuple[str, dict]] = [
    # # Octomap resolution — affects map fidelity for both planners
    # ("res_high",     {"octomap_resolution": "0.02"}),
    # # Semantic mismatch penalty — penalises label disagreement in the semantic map
    # ("penalty_low",  {"semantic_mismatch_penalty": "0.1"}),  # default 0.2
    # ("penalty_high", {"semantic_mismatch_penalty": "0.4"}),  # default 0.2
    # # Semantic confidence boost — rewards label agreement
    # ("boost_low",    {"semantic_confidence_boost": "0.15"}), # default 0.3
    # ("boost_high",   {"semantic_confidence_boost": "0.5"}),  # default 0.3
    # # Detection confidence threshold — controls which detections enter the map
    # ("conf_high",    {"conf_thresh": "0.10"}),
    # Camera max range — reduces stereo depth noise by cutting far detections
    # ("range_high",    {"camera_max_range": "0.9"}),
]

for _suffix, _overrides in _ABLATION_PASSES:
    for _tree_id, _baseline_bag, _semantic_bag in _ABLATION_TREES:
        _gt = f"metrics/penn_state/tree_{_tree_id}/gt_points.json"
        EXPERIMENT_CONFIGS.append((
            _baseline_bag,
            f"tree_{_tree_id}/baseline_{_suffix}/",
            _gt,
            {"decimation_factor": "2", **_overrides},
        ))
        EXPERIMENT_CONFIGS.append((
            _semantic_bag,
            f"tree_{_tree_id}/semantic_{_suffix}/",
            _gt,
            _overrides,
        ))

# NBV demo launch parameters (these stay the same across runs)
NBV_BASE_PARAMS = {
    "use_sim_time": "true",
    "use_gazebo": "false",
    "planner_type": "replay",
    "octomap_resolution": "0.04",
    # "camera_scaled_width": "896",
    # "camera_scaled_height": "672",
    # "stereo_matcher_model_trt": "fs_672x896_vit-small_iters16.plan",
    "camera_scaled_width": "448",
    "camera_scaled_height": "224",
    "stereo_matcher_model_trt": "fs_224x448_vit-small_iters5.plan",
    "detection_model_trt": "yolo26_large_seg_rivendale_v6_fold1.plan",
    "semantic_mismatch_penalty": "0.2",
    "semantic_confidence_boost": "0.3",
    "camera_max_range": "0.9",
    "export_viewpoint_voxels": "true",
    "metrics_dir": "metrics/penn_state",
    "conf_thresh": "0.05",
}

# rosbag_playback.launch.py publishes the robot's static TF tree via
# robot_state_publisher (plus RViz + odom_to_wheel_joints). Without it, incoming
# clouds can't be transformed into map_frame, the octomap never builds, and
# evaluation_metrics.json is never written.

# Rosbag topics to play back
ROSBAG_TOPICS = [
    "/tf",
    "/firefly_left/image_raw",
    "/firefly_left/camera_info",
    "/firefly_right/image_raw",
    "/firefly_right/camera_info",
]

# After each experiment, regenerate these plots from the tree's metrics.
# Each entry is (metric_name, output_filename_suffix). Output files are named
# <study_dir>/<tree>_<suffix>.png, e.g. metrics/penn_state/tree_11/tree_11_f1.png
PLOT_METRICS = [
    ("F1_Score", "f1"),
    ("mAP", "map"),
    ("Coverage_Percent", "coverage"),
]

# replot_metrics.py lives next to this script.
REPLOT_SCRIPT = str(Path(__file__).resolve().parent / "replot_metrics.py")

# Rosbag playback rate multiplier (1.0 = real-time, 2.0 = 2x speed, etc.).
# All sim-time-driven nodes (octomap, evaluator) scale automatically via /clock.
# Note: the stereo matcher and YOLO detector are compute-bound — at rates above
# ~2x you may start dropping frames if the GPU can't keep up.
ROSBAG_PLAYBACK_RATE = 2.0

# Seconds to wait for nbv_demo to initialize before starting the rosbag
NBV_STARTUP_WAIT = 5

# Seconds to wait after killing nbv_demo, so all nodes fully die before the
# next experiment starts (prevents demos accumulating across runs)
CLEANUP_WAIT = 10

# Seconds to wait for a graceful (SIGINT) shutdown before escalating
GRACEFUL_SHUTDOWN_TIMEOUT = 15

# Pre-flight check: how long to wait/retry for leftover nodes to clear before
# giving up and aborting an experiment. Set NODE_CHECK_RETRIES = 0 to disable.
NODE_CHECK_RETRIES = 6
NODE_CHECK_RETRY_WAIT = 5

# ROS nodes that are always present and are NOT considered "leftovers".
BENIGN_NODES = {
    "/launch_ros",  # ros2 launch's own helper node (prefix match, see below)
    "/_ros2cli",    # ros2 CLI helper nodes (prefix match)
    "/rviz",        # rviz, if you keep it open across runs (prefix match)
}

# ============================================================================
# PROCESS MANAGEMENT
# ============================================================================


def build_nbv_launch_command(run_name: str, gt_points_file: str, extra_params: dict = {}) -> List[str]:
    """Build the ros2 launch command for nbv_demo with given parameters."""
    cmd = ["ros2", "launch", "erwinia_os_nbv_planner", "nbv_demo.launch.py"]
    for key, value in {**NBV_BASE_PARAMS, **extra_params}.items():
        cmd.append(f"{key}:={value}")
    cmd.append(f"run:={run_name}")
    cmd.append(f"gt_points_file:={gt_points_file}")
    return cmd


def build_playback_launch_command() -> List[str]:
    """Build the ros2 launch command for rosbag_playback (robot TF tree + RViz)."""
    return ["ros2", "launch", "erwinia_os_description", "rosbag_playback.launch.py"]


def build_rosbag_command(rosbag_path: str) -> List[str]:
    """Build the ros2 bag play command."""
    cmd = ["ros2", "bag", "play", rosbag_path, "--clock",
           "--rate", str(ROSBAG_PLAYBACK_RATE), "--topics"]
    cmd.extend(ROSBAG_TOPICS)
    return cmd


def start_process(cmd: List[str], log_path: Optional[str] = None) -> subprocess.Popen:
    """Start a process in its own session/process group so the whole tree
    (including child nodes spawned by ros2 launch) can be signaled together.

    If log_path is given, stdout and stderr are tee'd to that file so warnings
    emitted by ROS nodes can be inspected after the run.
    """
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path, "w")
        return subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=log_file,
            start_new_session=True,
        )
    return subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,  # new process group; pgid == pid
    )


def shutdown_process(proc: Optional[subprocess.Popen], name: str) -> None:
    """Cleanly shut down a process and ALL of its children.

    Escalation: SIGINT (graceful, like Ctrl+C) -> SIGTERM -> SIGKILL,
    each sent to the entire process group.
    """
    if proc is None or proc.poll() is not None:
        return

    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    # 1. SIGINT to the whole group: ros2 launch handles this by shutting
    #    its nodes down cleanly.
    print(f"  Sending SIGINT to {name} process group (pgid={pgid})...")
    _signal_group(pgid, signal.SIGINT)
    if _wait_for_exit(proc, GRACEFUL_SHUTDOWN_TIMEOUT):
        print(f"  {name} shut down cleanly")
        return

    # 2. SIGTERM
    print(f"  {name} did not exit on SIGINT; sending SIGTERM...")
    _signal_group(pgid, signal.SIGTERM)
    if _wait_for_exit(proc, 5):
        print(f"  {name} terminated")
        return

    # 3. SIGKILL
    print(f"  {name} still alive; sending SIGKILL...")
    _signal_group(pgid, signal.SIGKILL)
    _wait_for_exit(proc, 5)
    print(f"  {name} killed")


def _signal_group(pgid: int, sig: int) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


def _wait_for_exit(proc: subprocess.Popen, timeout: float) -> bool:
    """Return True if the process exited within the timeout."""
    try:
        proc.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def list_leftover_nodes() -> List[str]:
    """Return ROS nodes currently in the graph that aren't benign helpers.

    Uses `ros2 node list`. Any node here before an experiment means a previous
    run did not fully shut down, which is what causes demos to accumulate.
    """
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"  WARNING: could not run 'ros2 node list' ({e}); skipping check")
        return []

    nodes = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return [n for n in nodes if not any(n.startswith(b) for b in BENIGN_NODES)]


def ensure_clean_node_graph() -> bool:
    """Wait for the ROS graph to be clear of leftover nodes.

    Returns True if the graph is clean (safe to start), False if leftover
    nodes persisted after all retries.
    """
    if NODE_CHECK_RETRIES <= 0:
        return True

    for attempt in range(NODE_CHECK_RETRIES + 1):
        leftovers = list_leftover_nodes()
        if not leftovers:
            return True
        print(
            f"  Pre-flight: {len(leftovers)} leftover node(s) still running: "
            f"{', '.join(leftovers)}"
        )
        if attempt < NODE_CHECK_RETRIES:
            print(f"  Waiting {NODE_CHECK_RETRY_WAIT}s for them to clear "
                  f"(attempt {attempt + 1}/{NODE_CHECK_RETRIES})...")
            time.sleep(NODE_CHECK_RETRY_WAIT)

    print("  ERROR: leftover nodes did not clear. Aborting to avoid mixing runs.")
    print("  Clean up manually with:  pkill -f ros2  &&  ros2 daemon stop")
    return False


# ============================================================================
# LOG SCANNING
# ============================================================================

# Lines from the nbv_demo log that contain these substrings are surfaced after
# each experiment. The GT_BBOX_WARNING tag is emitted by the evaluator when a
# GT segment lies outside the octomap bounding box.
LOG_WARN_PATTERNS = ["WARN", "ERROR"]


def report_run_warnings(log_path: str) -> int:
    """Print any warning/error lines from a run log. Returns the count found."""
    try:
        with open(log_path) as f:
            lines = f.readlines()
    except OSError:
        return 0

    hits = [l.rstrip() for l in lines if any(p in l for p in LOG_WARN_PATTERNS)]
    if hits:
        print(f"\n  [nbv_demo log warnings from {log_path}]")
        for line in hits:
            print(f"    {line}")
    return len(hits)


# ============================================================================
# EXPERIMENT RUNNER
# ============================================================================


def run_experiment(rosbag_path: str, run_name: str, gt_points_file: str, extra_params: dict = {}) -> bool:
    """Run a single experiment. Returns True on success."""
    print(f"\n{'='*70}")
    print(f"Starting experiment: {run_name}")
    print(f"Rosbag: {rosbag_path}")
    print(f"GT Points: {gt_points_file}")
    if extra_params:
        print(f"Extra params: {extra_params}")
    print(f"{'='*70}\n")

    if not Path(rosbag_path).exists():
        print(f"ERROR: Rosbag path does not exist: {rosbag_path}")
        return False

    # Pre-flight: make sure no nodes from a previous run are still alive.
    print("[1/6] Checking for leftover ROS nodes...")
    if not ensure_clean_node_graph():
        return False
    print("  Node graph is clean\n")

    # Log file for nbv_demo stdout+stderr; warnings are surfaced after the run.
    nbv_log_path = str(
        Path(NBV_BASE_PARAMS["metrics_dir"]) / run_name.strip("/") / "nbv_demo.log"
    )
    print(f"  nbv_demo log: {nbv_log_path}\n")

    playback_process: Optional[subprocess.Popen] = None
    nbv_process: Optional[subprocess.Popen] = None
    rosbag_process: Optional[subprocess.Popen] = None

    try:
        # Step 1: Launch NBV demo
        nbv_cmd = build_nbv_launch_command(run_name, gt_points_file, extra_params)
        print("[2/6] Launching nbv_demo...")
        print(f"  Command: {' '.join(nbv_cmd)}\n")
        nbv_process = start_process(nbv_cmd, log_path=nbv_log_path)

        # Step 2: Wait for NBV to initialize
        print(f"[3/6] Waiting {NBV_STARTUP_WAIT}s for nbv_demo to initialize...")
        time.sleep(NBV_STARTUP_WAIT)
        if nbv_process.poll() is not None:
            print("ERROR: nbv_demo exited prematurely")
            report_run_warnings(nbv_log_path)
            return False
        print(f"  nbv_demo is running (PID: {nbv_process.pid})\n")

        # Step 3: Launch rosbag_playback (robot_state_publisher -> TF tree + RViz).
        # Started after nbv_demo but before the bag, so the octomap can build.
        playback_cmd = build_playback_launch_command()
        print("[4/6] Launching rosbag_playback (robot TF tree + RViz)...")
        print(f"  Command: {' '.join(playback_cmd)}\n")
        playback_process = start_process(playback_cmd)
        time.sleep(NBV_STARTUP_WAIT)
        if playback_process.poll() is not None:
            print("ERROR: rosbag_playback exited prematurely")
            report_run_warnings(nbv_log_path)
            return False
        print(f"  rosbag_playback is running (PID: {playback_process.pid})\n")

        # Step 4: Start rosbag playback
        rosbag_cmd = build_rosbag_command(rosbag_path)
        print("[5/6] Starting rosbag playback...")
        print(f"  Command: {' '.join(rosbag_cmd)}\n")
        rosbag_process = start_process(rosbag_cmd)

        # Step 5: Wait for the rosbag to finish, then shut everything down
        print("[6/6] Waiting for rosbag to complete...")
        rosbag_process.wait()
        print("  Rosbag playback completed\n")

        print("Stopping rosbag_playback and nbv_demo...")
        shutdown_process(playback_process, "rosbag_playback")
        shutdown_process(nbv_process, "nbv_demo")
        report_run_warnings(nbv_log_path)
        return True

    finally:
        # Always make sure nothing is left running, even on exception/Ctrl+C.
        shutdown_process(rosbag_process, "rosbag")
        shutdown_process(nbv_process, "nbv_demo")
        shutdown_process(playback_process, "rosbag_playback")


def generate_plots(run_name: str) -> None:
    """Regenerate the configured plots for the tree this run belongs to.

    The study dir is the tree directory (parent of the run), e.g. a run of
    'tree_11/baseline/' produces plots under '<metrics_dir>/tree_11/'.
    """
    tree = run_name.strip("/").split("/")[0]  # e.g. "tree_11"
    study_dir = os.path.join(NBV_BASE_PARAMS["metrics_dir"], tree)

    print(f"Generating plots for {tree} (study: {study_dir})...")
    for metric, suffix in PLOT_METRICS:
        output = os.path.join(study_dir, "plots", f"{tree}_{suffix}.png")
        cmd = [
            "python3", REPLOT_SCRIPT,
            "--study", study_dir,
            "--output-kind", "plot",
            "--metrics", metric,
            "--output", output,
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"  {metric} -> {output}")
        else:
            print(f"  WARNING: plot for {metric} failed:\n{result.stderr.strip()}")


def main() -> int:
    print("\n" + "=" * 70)
    print("NBV Planner Experiment Runner")
    print("=" * 70)
    print(f"Total experiments to run: {len(EXPERIMENT_CONFIGS)}\n")

    if not EXPERIMENT_CONFIGS:
        print("ERROR: No experiment configurations defined!")
        return 1

    successful = 0
    failed = 0

    for i, (rosbag_path, run_name, gt_points_file, extra_params) in enumerate(
        EXPERIMENT_CONFIGS, 1
    ):
        print(f"\n[Experiment {i}/{len(EXPERIMENT_CONFIGS)}]")

        if run_experiment(rosbag_path, run_name, gt_points_file, extra_params):
            successful += 1
            generate_plots(run_name)
        else:
            failed += 1

        # Wait for all nodes to fully die before the next experiment.
        if i < len(EXPERIMENT_CONFIGS):
            print(f"Waiting {CLEANUP_WAIT}s for cleanup before next experiment...")
            time.sleep(CLEANUP_WAIT)

    print("\n" + "=" * 70)
    print("EXPERIMENT RUN COMPLETE")
    print("=" * 70)
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")
    print(f"Total: {len(EXPERIMENT_CONFIGS)}\n")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # run_experiment's finally block already shuts down the current
        # processes; this just reports the interruption.
        print("\n\nExperiment run interrupted by user")
        sys.exit(1)
