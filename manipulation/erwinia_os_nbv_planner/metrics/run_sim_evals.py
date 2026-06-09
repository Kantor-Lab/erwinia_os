#!/usr/bin/env python3
"""
Orchestrate NBV planner experiments in live Gazebo simulation.

Sequence for each (tree, rotation) session:
1. Launch bringup (Gazebo + robot + MoveIt)
2. Wait for environment to initialize
3. For each planner (baseline, volumetric, semantic):
   a. Launch nbv_demo
   b. Wait for it to finish naturally (exits after max_iterations)
   c. Brief cleanup wait
4. Shut down bringup (whole process group)
5. Wait for all nodes to die, then advance to the next rotation/tree

Each of the 5 simulated trees is run at 12 yaw orientations (0°–330°, every 30°),
giving 12 × 5 × 3 planners = 180 experiments total. The GT points file is always
read from the original tree_N folder; output is saved to tree_N_Xdeg/ directories.

KEY DETAIL: `ros2 launch` spawns child node processes. Killing only the launch
process orphans those nodes. Every subprocess is started in its own session/process
group via start_new_session=True, and we signal the ENTIRE group with os.killpg().
"""

import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ============================================================================
# CONFIGURATION
# ============================================================================

# (tree_id, world_name, position_str, gt_points_file)
TREE_CONFIGS: List[Tuple[int, str, str, str]] = [
    (1, "apple_tree_1", "0 -1.6 0", "metrics/blender/tree_1/gt_points.json"),
    (2, "apple_tree_2", "0 -1.6 0", "metrics/blender/tree_2/gt_points.json"),
    (3, "apple_tree_3", "0 -1.6 0", "metrics/blender/tree_3/gt_points.json"),
    (4, "apple_tree_4", "0 -1.6 0", "metrics/blender/tree_4/gt_points.json"),
    (5, "apple_tree_5", "0 -1.6 0", "metrics/blender/tree_5/gt_points.json"),
]

# Step size (degrees) between rotation experiments. 360 must be divisible by this.
ROTATION_STEP_DEG: int = 0

# Additional yaw angles to sweep (degrees). The base 1.5708 rad aligns the
# Blender-exported model with Gazebo's world axes; each step rotates on top of that.
ROTATION_DEGREES: List[int] = [0] if ROTATION_STEP_DEG in (0, 360) else list(range(0, 360, ROTATION_STEP_DEG))

# Yaw correction for Blender→Gazebo coordinate system (π/2). gt_points_rotation='0 0 0'
# aligns GT points with the mesh at this yaw; additional steps co-rotate both.
BASE_YAW_RAD: float = 1.5708

# Planners to run per Gazebo session, in order: (planner_type, run_label, extra_params)
# run_label is used for the output subdirectory and log path; planner_type is the
# actual launch argument. They differ for ablation runs.
PLANNERS: List[Tuple[str, str, Dict[str, str]]] = [
    ("baseline",   "baseline",   {}),
    ("volumetric", "volumetric", {}),
    ("semantic",   "semantic",   {}),
]

# ============================================================================
# ABLATION PASSES — each entry is (suffix, param_overrides).
# Generates additional baseline + semantic runs per session with one param varied.
# Volumetric is omitted (no semantic params to ablate).
# ============================================================================
_ABLATION_PASSES: List[Tuple[str, Dict[str, str]]] = [
    # ("res_high",   {"octomap_resolution": "0.02"}),
    # ("conf_high",  {"conf_thresh": "0.10"}),
    # ("range_high", {"camera_max_range": "0.9"}),
    # ("beta_low",   {"beta_semantic_weight": "0.3"}),  # default 0.5
    # ("beta_high",  {"beta_semantic_weight": "0.7"}),  # default 0.5
    # # Semantic mismatch penalty — penalises label disagreement in the semantic map
    # ("penalty_low",  {"semantic_mismatch_penalty": "0.1"}),  # default 0.2
    # ("penalty_high", {"semantic_mismatch_penalty": "0.4"}),  # default 0.2
    # # Semantic confidence boost — rewards label agreement
    # ("boost_low",    {"semantic_confidence_boost": "0.15"}), # default 0.3
    # ("boost_high",   {"semantic_confidence_boost": "0.5"}),  # default 0.3
]

for _suffix, _overrides in _ABLATION_PASSES:
    PLANNERS.append(("volumetric", f"volumetric_{_suffix}", _overrides))
    PLANNERS.append(("semantic",   f"semantic_{_suffix}",   _overrides))

# NBV demo base parameters shared across all sim runs
NBV_BASE_PARAMS: Dict[str, str] = {
    "use_gazebo":                "true",
    "use_sim_time":              "true",
    "max_iterations":            "30",
    "detection_model_trt":       "best_sim_seg_v2.plan",
    "conf_thresh":               "0.1",
    "export_viewpoint_voxels":   "true",
    "octomap_resolution":        "0.02",
    "camera_max_range":          "0.7",
    "semantic_mismatch_penalty": "0.2",
    "semantic_confidence_boost": "0.3",
    "beta_semantic_weight":      "0.5",
    "octomap_use_moveit":        "true",
}

# How long to wait for Gazebo + robot stack to be ready before launching planners
BRINGUP_STARTUP_WAIT: int = 30

# How long to wait for nbv_demo nodes to initialize before checking they're alive
NBV_STARTUP_WAIT: int = 5

# How long to wait after each nbv_demo finishes (lets nodes die before next planner)
PLANNER_CLEANUP_WAIT: int = 10

# How long to wait after shutting down bringup before the next Gazebo relaunch
BRINGUP_CLEANUP_WAIT: int = 20

# Graceful (SIGINT) shutdown timeout before escalating to SIGTERM/SIGKILL
GRACEFUL_SHUTDOWN_TIMEOUT: int = 15

# Pre-flight node-graph cleanliness check
NODE_CHECK_RETRIES: int = 6
NODE_CHECK_RETRY_WAIT: int = 5

# Nodes always present that are NOT considered leftover from a previous run
BENIGN_NODES = {"/launch_ros", "/_ros2cli", "/rviz"}

# Log lines containing these strings are surfaced after each planner run
LOG_WARN_PATTERNS = ["WARN", "ERROR"]

# Metrics to plot after each session. (metric_name, output_filename_suffix)
PLOT_METRICS = [
    ("F1_Score",         "f1"),
    ("mAP",              "map"),
    ("Coverage_Percent", "coverage"),
]

REPLOT_SCRIPT = str(Path(__file__).resolve().parent / "replot_metrics.py")

# ============================================================================
# COMMAND BUILDERS
# ============================================================================


def build_bringup_command(world: str, position: str, rotation: str) -> List[str]:
    return [
        "ros2", "launch", "erwinia_os_bringup", "bringup.launch.py",
        "use_gazebo:=true",
        "use_sim_time:=true",
        f"world:={world}",
        f"position:={position}",
        f"rotation:={rotation}",
    ]


def build_nbv_command(
    planner_type: str,
    run_name: str,
    gt_points_file: str,
    gt_points_position: str,
    gt_points_rotation: str,
    metrics_dir: str,
    extra_params: Dict[str, str],
) -> List[str]:
    cmd = ["ros2", "launch", "erwinia_os_nbv_planner", "nbv_demo.launch.py"]
    params = {
        **NBV_BASE_PARAMS,
        **extra_params,
        "planner_type":       planner_type,
        "run":                run_name,
        "gt_points_file":     gt_points_file,
        "gt_points_position": gt_points_position,
        "gt_points_rotation": gt_points_rotation,
        "metrics_dir":        metrics_dir,
    }
    for key, value in params.items():
        cmd.append(f"{key}:={value}")
    return cmd


# ============================================================================
# PROCESS MANAGEMENT (mirrored from run_rosbag_evals.py)
# ============================================================================


def start_process(cmd: List[str], log_path: Optional[str] = None) -> subprocess.Popen:
    """Start a process in its own session so the whole tree can be signaled together."""
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path, "w")
        return subprocess.Popen(cmd, stdout=log_file, stderr=log_file,
                                start_new_session=True)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)


def shutdown_process(proc: Optional[subprocess.Popen], name: str) -> None:
    """Cleanly shut down a process and ALL of its children (SIGINT → SIGTERM → SIGKILL)."""
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    print(f"  Sending SIGINT to {name} process group (pgid={pgid})...")
    _signal_group(pgid, signal.SIGINT)
    if _wait_for_exit(proc, GRACEFUL_SHUTDOWN_TIMEOUT):
        print(f"  {name} shut down cleanly")
        return

    print(f"  {name} did not exit on SIGINT; sending SIGTERM...")
    _signal_group(pgid, signal.SIGTERM)
    if _wait_for_exit(proc, 5):
        print(f"  {name} terminated")
        return

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
    try:
        proc.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def list_leftover_nodes() -> List[str]:
    """Return ROS nodes currently running that aren't benign infrastructure."""
    try:
        result = subprocess.run(["ros2", "node", "list"],
                                capture_output=True, text=True, timeout=15)
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"  WARNING: could not run 'ros2 node list' ({e}); skipping check")
        return []
    nodes = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return [n for n in nodes if not any(n.startswith(b) for b in BENIGN_NODES)]


def ensure_clean_node_graph() -> bool:
    """Wait for the ROS graph to clear. Returns True if clean, False if timed out."""
    if NODE_CHECK_RETRIES <= 0:
        return True
    for attempt in range(NODE_CHECK_RETRIES + 1):
        leftovers = list_leftover_nodes()
        if not leftovers:
            return True
        print(f"  Pre-flight: {len(leftovers)} leftover node(s): {', '.join(leftovers)}")
        if attempt < NODE_CHECK_RETRIES:
            print(f"  Waiting {NODE_CHECK_RETRY_WAIT}s "
                  f"(attempt {attempt + 1}/{NODE_CHECK_RETRIES})...")
            time.sleep(NODE_CHECK_RETRY_WAIT)
    print("  ERROR: leftover nodes did not clear. Aborting.")
    print("  Clean up manually with:  pkill -f ros2  &&  ros2 daemon stop")
    return False


# ============================================================================
# LOG SCANNING
# ============================================================================


def report_run_warnings(log_path: str) -> int:
    """Print warning/error lines from a run log. Returns the count found."""
    try:
        with open(log_path) as f:
            lines = f.readlines()
    except OSError:
        return 0
    hits = [l.rstrip() for l in lines if any(p in l for p in LOG_WARN_PATTERNS)]
    if hits:
        print(f"\n  [log warnings from {log_path}]")
        for line in hits:
            print(f"    {line}")
    return len(hits)


# ============================================================================
# PLOT GENERATION
# ============================================================================


def generate_plots(tree_id: int, deg: int) -> None:
    """Regenerate configured plots for a completed session's study directory."""
    tree_dir = f"tree_{tree_id}" if deg == 0 else f"tree_{tree_id}_{deg}"
    study_dir = f"metrics/blender/{tree_dir}"

    print(f"Generating plots for {tree_dir} (study: {study_dir})...")
    for metric, suffix in PLOT_METRICS:
        output = os.path.join(study_dir, "plots", f"{tree_dir}_{suffix}.png")
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


# ============================================================================
# EXPERIMENT RUNNERS
# ============================================================================


def run_single_planner(
    planner_type: str,
    run_label: str,
    extra_params: Dict[str, str],
    tree_id: int,
    deg: int,
    gt_points_file: str,
    gt_points_position: str,
    gt_points_rotation: str,
) -> bool:
    """Launch one planner, wait for natural exit, return True on success."""
    tree_dir = f"tree_{tree_id}" if deg == 0 else f"tree_{tree_id}_{deg}"
    metrics_dir = f"metrics/blender/{tree_dir}"
    log_path = f"{metrics_dir}/{run_label}/nbv_demo.log"

    nbv_cmd = build_nbv_command(
        planner_type=planner_type,
        run_name=run_label,
        gt_points_file=gt_points_file,
        gt_points_position=gt_points_position,
        gt_points_rotation=gt_points_rotation,
        metrics_dir=metrics_dir,
        extra_params=extra_params,
    )

    print(f"\n  Launching {run_label} planner...")
    print(f"  Command: {' '.join(nbv_cmd)}")
    print(f"  Log: {log_path}\n")

    nbv_process: Optional[subprocess.Popen] = None
    try:
        nbv_process = start_process(nbv_cmd, log_path=log_path)

        time.sleep(NBV_STARTUP_WAIT)
        if nbv_process.poll() is not None:
            print(f"  ERROR: {run_label} nbv_demo exited prematurely")
            report_run_warnings(log_path)
            return False

        print(f"  {run_label} running (PID: {nbv_process.pid}). "
              "Waiting for completion...")
        nbv_process.wait()

        rc = nbv_process.returncode
        if rc != 0:
            print(f"  WARNING: {run_label} exited with code {rc}")

        report_run_warnings(log_path)
        print(f"  {run_label} complete")
        return rc == 0

    finally:
        shutdown_process(nbv_process, f"nbv_demo ({run_label})")


def run_gazebo_session(
    tree_id: int,
    world: str,
    position: str,
    gt_points_file: str,
    deg: int,
) -> bool:
    """
    Launch one Gazebo session for the given tree at the given rotation,
    run all three planners in sequence, then tear down Gazebo.
    Returns True if all planners succeeded.
    """
    extra_yaw_rad = math.radians(deg)
    mesh_rotation = f"0 0 {BASE_YAW_RAD + extra_yaw_rad:.6f}"
    gt_points_rotation = f"0 0 {extra_yaw_rad:.6f}"

    print(f"\n{'='*70}")
    print(f"Tree {tree_id} | Rotation {deg}°")
    print(f"  World:          {world}")
    print(f"  Mesh rotation:  {mesh_rotation}")
    print(f"  GT rotation:    {gt_points_rotation}")
    print(f"  GT position:    {position}")
    tree_dir = f"tree_{tree_id}" if deg == 0 else f"tree_{tree_id}_{deg}"
    print(f"  Output dir:     metrics/blender/{tree_dir}/")
    print(f"{'='*70}\n")

    print("[1/4] Checking for leftover ROS nodes...")
    if not ensure_clean_node_graph():
        return False
    print("  Node graph is clean\n")

    bringup_process: Optional[subprocess.Popen] = None
    all_succeeded = True

    try:
        bringup_cmd = build_bringup_command(world, position, mesh_rotation)
        print("[2/4] Launching bringup (Gazebo + robot)...")
        print(f"  Command: {' '.join(bringup_cmd)}\n")
        bringup_process = start_process(bringup_cmd)

        print(f"  Waiting {BRINGUP_STARTUP_WAIT}s for environment to initialize...")
        time.sleep(BRINGUP_STARTUP_WAIT)

        if bringup_process.poll() is not None:
            print("  ERROR: bringup exited prematurely — skipping all planners")
            return False
        print(f"  Bringup running (PID: {bringup_process.pid})\n")

        print("[3/4] Running planners...")
        for planner_type, run_label, extra_params in PLANNERS:
            success = run_single_planner(
                planner_type=planner_type,
                run_label=run_label,
                extra_params=extra_params,
                tree_id=tree_id,
                deg=deg,
                gt_points_file=gt_points_file,
                gt_points_position=position,
                gt_points_rotation=gt_points_rotation,
            )
            if not success:
                all_succeeded = False

            generate_plots(tree_id, deg)

            if PLANNER_CLEANUP_WAIT > 0:
                print(f"  Waiting {PLANNER_CLEANUP_WAIT}s before next planner...")
                time.sleep(PLANNER_CLEANUP_WAIT)

        print("\n[4/4] Shutting down bringup...")
        shutdown_process(bringup_process, "bringup")
        bringup_process = None

        print(f"  Waiting {BRINGUP_CLEANUP_WAIT}s for full teardown...")
        time.sleep(BRINGUP_CLEANUP_WAIT)

        print("  Verifying clean shutdown...")
        ensure_clean_node_graph()

        return all_succeeded

    finally:
        shutdown_process(bringup_process, "bringup")


# ============================================================================
# MAIN
# ============================================================================


def main() -> int:
    total = len(TREE_CONFIGS) * len(ROTATION_DEGREES)
    planner_total = total * len(PLANNERS)

    print("\n" + "=" * 70)
    print("NBV Sim Experiment Runner")
    print("=" * 70)
    print(f"Trees:      {len(TREE_CONFIGS)}")
    print(f"Rotations:  {len(ROTATION_DEGREES)} ({ROTATION_DEGREES}°)")
    print(f"Planners:   {len(PLANNERS)} ({[label for _, label, _ in PLANNERS]})")
    print(f"Sessions:   {total}  |  Total planner runs: {planner_total}\n")

    successful_sessions = 0
    failed_sessions = 0
    session_num = 0

    for tree_id, world, position, gt_points_file in TREE_CONFIGS:
        for deg in ROTATION_DEGREES:
            session_num += 1
            print(f"\n[Session {session_num}/{total}]")

            if run_gazebo_session(tree_id, world, position, gt_points_file, deg):
                successful_sessions += 1
            else:
                failed_sessions += 1

    print("\n" + "=" * 70)
    print("SIM EXPERIMENT RUN COMPLETE")
    print("=" * 70)
    print(f"Successful sessions: {successful_sessions}")
    print(f"Failed sessions:     {failed_sessions}")
    print(f"Total sessions:      {total}\n")

    return 0 if failed_sessions == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nExperiment run interrupted by user")
        sys.exit(1)
