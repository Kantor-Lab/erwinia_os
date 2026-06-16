#!/usr/bin/env python3
"""
Plot or tabulate NBV metrics from current evaluation_metrics.json artifacts.

Inputs can be individual JSON files, run directories containing
evaluation_metrics.json, planner study directories, or a metrics root containing
tree directories. The script writes either a plot or a table, selected
explicitly with --output-kind.

Common examples:
  # Plot one JSON run.
  python3 replot_metrics.py --runs metrics/penn_state/tree_11/semantic --output-kind plot \
      --metrics mAP F1_Score Coverage_Percent --output semantic.png

  # Table at viewpoint 20 for one study; each planner is pooled into one row.
  python3 replot_metrics.py --study metrics/penn_state/tree_11 --output-kind table \
      --metrics mAP F1_Score Coverage_Percent --table 20 --output tree_11_summary.csv

  # Plot pooled planner curves across all trees under a metrics root.
  python3 replot_metrics.py --metrics-root metrics/penn_state --output-kind plot \
      --metrics mAP F1_Score Coverage_Percent --output planner_means.png

  # Time-axis plot sampled every 5 seconds.
  python3 replot_metrics.py --runs metrics/penn_state/tree_11/semantic --output-kind plot \
      --x time --x-step 5 --metrics mAP F1_Score --output semantic_time.png

JSON metric definitions:
  Precision, Recall, F1_Score, TP_Clusters, FP_Clusters, and FN_Clusters are
  recomputed from groundTruthSegments, predictedClusters, and pairwiseDistances.
  By default F1_Score is the maximum F1 achieved across all observed detection
  confidence thresholds. With --f1-confidence, the score is instead computed at
  that fixed minimum confidence.

  Matching uses a three-stage algorithm at each distance threshold:
    1. Greedy one-to-one matching: predictions sorted by confidence are matched
       to the nearest unmatched same-class GT within --distance-threshold meters.
    2. Split credit (--disable-cluster-splitting to turn off): each matched
       prediction also credits any additional unmatched GTs within threshold,
       reducing FNs without creating FPs.
    3. Merge cleanup (--disable-cluster-merging to turn off): unmatched
       predictions within threshold of an already-TP GT are treated as redundant
       coverage and removed from the FP count.

  mAP is computed like COCO detection AP but with a 3D distance threshold
  instead of IoU. By default AP is averaged over thresholds 0.20, 0.18, ...,
  0.02 m and over all classes that have ground truth or predictions. Classes
  with predictions but no ground truth contribute AP = 0. The same three-stage
  matching applies at each threshold.

  Coverage_Percent, Occupied_Voxels, Free_Voxels, and Total_Voxels are read or
  derived directly from the per-viewpoint JSON fields.

X-axis modes (--x):
  viewpoint  One point per viewpoint index (default). Use --x-step N to sample
             every Nth viewpoint.
  time       One point per integer second using zero-order hold: each second
             uses the predictions from the most recently completed viewpoint.
             Use --x-step N to sample every N seconds.
"""

import argparse
import json
import os
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd


JSON_METRICS_FILENAME = "evaluation_metrics.json"

DEFAULT_MAP_DISTANCE_THRESHOLD_MAX_M = 0.20
DEFAULT_MAP_DISTANCE_THRESHOLD_MIN_M = 0.02
DEFAULT_MAP_DISTANCE_THRESHOLD_STEP_M = 0.02

COUNT_METRICS = {
    "TP_Clusters",
    "FP_Clusters",
    "FN_Clusters",
}

DERIVED_METRICS = {
    "Precision",
    "Recall",
    "F1_Score",
    "mAP",
}

GLOBAL_METRICS = {
    "Coverage_Percent",
    "Occupied_Voxels",
    "Free_Voxels",
    "Total_Voxels",
}

AVAILABLE_METRICS = sorted(GLOBAL_METRICS | COUNT_METRICS | DERIVED_METRICS)

X_COLUMN_MAP = {
    "time": "Time",
    "viewpoint": "Viewpoint",
    "run": "Viewpoint",
}

JSON_KEY_MAP = {
    "Time": "timeSec",
    "Viewpoint": "viewpointIndex",
    "Coverage_Percent": "bboxCoverage",
    "Occupied_Voxels": "occupiedVoxels",
    "Free_Voxels": "freeVoxels",
}


def get_pyplot():
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    import matplotlib.pyplot as plt

    return plt


def safe_divide(num: float, den: float) -> float:
    return float(num) / float(den) if den and not pd.isna(den) else float("nan")


def compute_precision_recall_f1(tp_clusters: float, fp_clusters: float, fn_clusters: float) -> tuple[float, float, float]:
    predicted = tp_clusters + fp_clusters
    actual = tp_clusters + fn_clusters

    if predicted == 0 and actual > 0:
        return 0.0, 0.0, 0.0
    if predicted == 0 and actual == 0:
        return float("nan"), float("nan"), float("nan")
    if predicted > 0 and actual == 0:
        return 0.0, float("nan"), 0.0

    precision = safe_divide(tp_clusters, predicted)
    recall = safe_divide(tp_clusters, actual)
    if pd.isna(precision) or pd.isna(recall):
        return precision, recall, float("nan")
    if precision + recall == 0:
        return precision, recall, 0.0
    return precision, recall, 2.0 * precision * recall / (precision + recall)


def distance_thresholds_from_args(args: argparse.Namespace) -> list[float]:
    if args.map_thresholds:
        return args.map_thresholds
    if args.map_threshold_step <= 0:
        raise ValueError("--map-threshold-step must be positive.")
    values = []
    threshold = args.map_threshold_max
    while threshold >= args.map_threshold_min - 1e-9:
        values.append(round(threshold, 6))
        threshold -= args.map_threshold_step
    return values


def class_allowed(class_id: int, classes: Optional[list[int]]) -> bool:
    return classes is None or class_id in classes


def prediction_confidence(prediction: dict) -> float:
    for key in ("maxConfidence", "confidence", "score", "probability"):
        if key in prediction:
            return float(prediction[key])
    return 1.0


def build_prediction_lookup(viewpoint: dict, classes: Optional[list[int]]) -> dict[int, dict]:
    predictions = {}
    for pred in viewpoint.get("predictedClusters", []):
        label = pred.get("label")
        class_id = pred.get("classId")
        if label is None or class_id is None or not class_allowed(int(class_id), classes):
            continue
        predictions[int(label)] = pred
    return predictions


def best_distance_pairs(viewpoint: dict, classes: Optional[list[int]]) -> dict[tuple[str, int], float]:
    out = {}
    for pair in viewpoint.get("pairwiseDistances", []):
        gt_class = pair.get("gtClassId")
        pred_class = pair.get("predictedClassId")
        gt_id = pair.get("gtId")
        pred_label = pair.get("predictedLabel")
        distance = pair.get("distanceM")
        if gt_class is None or pred_class is None or gt_id is None or pred_label is None or distance is None:
            continue
        if int(gt_class) != int(pred_class):
            continue
        if not class_allowed(int(gt_class), classes):
            continue
        key = (str(gt_id), int(pred_label))
        out[key] = min(float(distance), out.get(key, float("inf")))
    return out


def match_predictions_at_threshold(
    viewpoint: dict,
    classes: Optional[list[int]],
    distance_threshold: float,
    min_confidence: float = 0.0,
    split_clusters: bool = False,
    merge_clusters: bool = False,
) -> tuple[int, int, int]:
    gt_ids = {
        str(gt["id"])
        for gt in viewpoint.get("groundTruthSegments", [])
        if "id" in gt and "classId" in gt and class_allowed(int(gt["classId"]), classes)
    }
    predictions = {
        label: pred
        for label, pred in build_prediction_lookup(viewpoint, classes).items()
        if prediction_confidence(pred) >= min_confidence
    }
    pair_distances = best_distance_pairs(viewpoint, classes)

    candidate_matches = []
    for (gt_id, pred_label), distance in pair_distances.items():
        if gt_id not in gt_ids or pred_label not in predictions or distance > distance_threshold:
            continue
        confidence = prediction_confidence(predictions[pred_label])
        candidate_matches.append((distance, -confidence, gt_id, pred_label))

    used_gt = set()
    used_pred = set()
    for _distance, _neg_confidence, gt_id, pred_label in sorted(candidate_matches):
        if gt_id in used_gt or pred_label in used_pred:
            continue
        used_gt.add(gt_id)
        used_pred.add(pred_label)

    # Split credit: each matched cluster also credits any additional nearby unmatched GTs.
    if split_clusters:
        for pred_label in list(used_pred):
            for gt_id in list(gt_ids - used_gt):
                if pair_distances.get((gt_id, pred_label), float("inf")) <= distance_threshold:
                    used_gt.add(gt_id)

    # Merge cleanup: unmatched predictions that are within threshold of an already-TP GT
    # are redundant coverage — don't count them as FPs.
    if merge_clusters:
        for pred_label in list(set(predictions) - used_pred):
            for gt_id in used_gt:
                if pair_distances.get((gt_id, pred_label), float("inf")) <= distance_threshold:
                    used_pred.add(pred_label)
                    break

    tp = len(used_gt)
    fp = len(predictions) - len(used_pred)
    fn = len(gt_ids) - len(used_gt)
    return tp, fp, fn


def compute_best_f1(
    viewpoint: dict,
    classes: Optional[list[int]],
    distance_threshold: float,
    fixed_confidence: Optional[float] = None,
    split_clusters: bool = False,
    merge_clusters: bool = False,
) -> tuple[int, int, int, float, float, float]:
    if fixed_confidence is not None:
        tp, fp, fn = match_predictions_at_threshold(
            viewpoint, classes, distance_threshold, min_confidence=fixed_confidence,
            split_clusters=split_clusters, merge_clusters=merge_clusters,
        )
        precision, recall, f1 = compute_precision_recall_f1(tp, fp, fn)
        return tp, fp, fn, precision, recall, f1

    predictions = build_prediction_lookup(viewpoint, classes)
    thresholds = sorted(
        {prediction_confidence(pred) for pred in predictions.values()},
        reverse=True,
    )

    if not thresholds:
        tp, fp, fn = match_predictions_at_threshold(
            viewpoint, classes, distance_threshold,
            split_clusters=split_clusters, merge_clusters=merge_clusters,
        )
        precision, recall, f1 = compute_precision_recall_f1(tp, fp, fn)
        return tp, fp, fn, precision, recall, f1

    best: Optional[tuple[int, int, int, float, float, float]] = None
    for threshold in thresholds:
        tp, fp, fn = match_predictions_at_threshold(
            viewpoint, classes, distance_threshold, min_confidence=threshold,
            split_clusters=split_clusters, merge_clusters=merge_clusters,
        )
        precision, recall, f1 = compute_precision_recall_f1(tp, fp, fn)
        if best is None or (not pd.isna(f1) and (pd.isna(best[5]) or f1 > best[5])):
            best = (tp, fp, fn, precision, recall, f1)

    assert best is not None
    return best



def compute_ap_for_class(
    viewpoint: dict,
    class_id: int,
    distance_threshold: float,
    split_clusters: bool = False,
    merge_clusters: bool = False,
) -> float:
    gt_ids = {
        str(gt["id"])
        for gt in viewpoint.get("groundTruthSegments", [])
        if "id" in gt and int(gt.get("classId", -1)) == class_id
    }

    predictions = [
        pred for pred in viewpoint.get("predictedClusters", [])
        if int(pred.get("classId", -1)) == class_id and pred.get("label") is not None
    ]

    if not gt_ids:
        # Predictions with no GT are all FP, so AP = 0; no predictions means the class is skipped.
        return 0.0 if predictions else float("nan")

    if not predictions:
        return 0.0

    pair_distances = best_distance_pairs(viewpoint, [class_id])
    sorted_predictions = sorted(predictions, key=prediction_confidence, reverse=True)

    used_gt = set()
    tp_flags = []
    fp_flags = []

    for pred in sorted_predictions:
        pred_label = int(pred["label"])
        best_gt = None
        best_dist = float("inf")
        for gt_id in gt_ids:
            if gt_id in used_gt:
                continue
            distance = pair_distances.get((gt_id, pred_label), float("inf"))
            if distance <= distance_threshold and distance < best_dist:
                best_gt = gt_id
                best_dist = distance

        if best_gt is not None:
            used_gt.add(best_gt)
            tp_flags.append(1.0)
            fp_flags.append(0.0)
            # Split credit: also credit any additional unmatched GTs this cluster covers.
            if split_clusters:
                for other_gt in list(gt_ids - used_gt):
                    d = pair_distances.get((other_gt, pred_label), float("inf"))
                    if d <= distance_threshold:
                        used_gt.add(other_gt)
                        tp_flags.append(1.0)
                        fp_flags.append(0.0)
        else:
            # Merge cleanup: skip predictions that are redundant near an already-TP GT.
            if merge_clusters and any(
                gt_id in used_gt
                and pair_distances.get((gt_id, pred_label), float("inf")) <= distance_threshold
                for gt_id in gt_ids
            ):
                continue
            tp_flags.append(0.0)
            fp_flags.append(1.0)

    tp_cum = np.cumsum(tp_flags)
    fp_cum = np.cumsum(fp_flags)
    recalls = tp_cum / max(len(gt_ids), 1)
    precisions = tp_cum / np.maximum(tp_cum + fp_cum, 1e-12)

    recall_points = np.concatenate(([0.0], recalls, [1.0]))
    precision_points = np.concatenate(([1.0], precisions, [0.0]))
    for i in range(len(precision_points) - 2, -1, -1):
        precision_points[i] = max(precision_points[i], precision_points[i + 1])

    changing = np.where(recall_points[1:] != recall_points[:-1])[0]
    return float(np.sum((recall_points[changing + 1] - recall_points[changing]) * precision_points[changing + 1]))


def compute_map(viewpoint: dict, classes: Optional[list[int]], distance_thresholds: list[float], split_clusters: bool = False, merge_clusters: bool = False) -> float:
    gt_classes = {
        int(gt["classId"])
        for gt in viewpoint.get("groundTruthSegments", [])
        if "classId" in gt and class_allowed(int(gt["classId"]), classes)
    }
    pred_classes = {
        int(pred["classId"])
        for pred in viewpoint.get("predictedClusters", [])
        if "classId" in pred and class_allowed(int(pred["classId"]), classes)
    }
    all_classes = sorted(gt_classes | pred_classes)
    ap_values = []
    for threshold in distance_thresholds:
        for class_id in all_classes:
            ap = compute_ap_for_class(viewpoint, class_id, threshold, split_clusters=split_clusters, merge_clusters=merge_clusters)
            if not pd.isna(ap):
                ap_values.append(ap)
    return float(np.mean(ap_values)) if ap_values else float("nan")



def find_json_metrics_file(path: Path) -> Optional[Path]:
    candidates = [
        path / "data" / JSON_METRICS_FILENAME,
        path / JSON_METRICS_FILENAME,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def discover_runs(container_dir: Path) -> dict[str, Path]:
    runs = {}
    direct = find_json_metrics_file(container_dir)
    if direct:
        runs[container_dir.name] = direct
        return runs

    for child in sorted(container_dir.iterdir()):
        if not child.is_dir():
            continue
        metrics_file = find_json_metrics_file(child)
        if metrics_file:
            runs[child.name] = metrics_file
    return runs


def discover_single_study(study_dir: Path) -> dict[str, dict[str, Path]]:
    planners = {}
    for planner_dir in sorted(study_dir.iterdir()):
        if not planner_dir.is_dir():
            continue
        runs = discover_runs(planner_dir)
        if runs:
            planners[planner_dir.name] = runs
    return planners


def discover_metrics_root(metrics_root: Path) -> dict[str, dict[str, dict[str, Path]]]:
    out = {}
    for tree_dir in sorted(metrics_root.iterdir()):
        if not tree_dir.is_dir():
            continue
        planners = discover_single_study(tree_dir)
        if planners:
            out[tree_dir.name] = planners
    return out


def _viewpoint_at_time(viewpoints: list[dict], t: float) -> Optional[dict]:
    """Return the last viewpoint whose timeSec <= t (zero-order hold), or None."""
    result = None
    for vp in viewpoints:
        ts = vp.get("timeSec")
        if ts is not None and ts <= t:
            result = vp
        elif ts is not None and ts > t:
            break
    return result


def load_json_run(
    json_path: Path,
    classes: Optional[list[int]],
    x_col: str,
    metrics: list[str],
    distance_threshold: float,
    map_thresholds: list[float],
    f1_confidence: Optional[float] = None,
    split_clusters: bool = True,
    merge_clusters: bool = True,
    x_step: int = 1,
) -> pd.DataFrame:
    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    viewpoints = data.get("viewpoints", [])

    def _compute_row(viewpoint: dict, x_val) -> dict:
        row: dict = {
            "Viewpoint": viewpoint.get(JSON_KEY_MAP["Viewpoint"]),
            "Time": x_val if x_col == "Time" else viewpoint.get(JSON_KEY_MAP["Time"]),
            "Coverage_Percent": viewpoint.get(JSON_KEY_MAP["Coverage_Percent"]),
            "Occupied_Voxels": viewpoint.get(JSON_KEY_MAP["Occupied_Voxels"]),
            "Free_Voxels": viewpoint.get(JSON_KEY_MAP["Free_Voxels"]),
        }
        if row["Occupied_Voxels"] is not None and row["Free_Voxels"] is not None:
            row["Total_Voxels"] = row["Occupied_Voxels"] + row["Free_Voxels"]
        if any(m in metrics for m in ("Precision", "Recall", "F1_Score", "TP_Clusters", "FP_Clusters", "FN_Clusters")):
            tp, fp, fn, precision, recall, f1 = compute_best_f1(
                viewpoint, classes, distance_threshold, fixed_confidence=f1_confidence,
                split_clusters=split_clusters, merge_clusters=merge_clusters,
            )
            row.update({"TP_Clusters": tp, "FP_Clusters": fp, "FN_Clusters": fn,
                        "Precision": precision, "Recall": recall, "F1_Score": f1})
        if "mAP" in metrics:
            row["mAP"] = compute_map(viewpoint, classes, map_thresholds,
                                     split_clusters=split_clusters, merge_clusters=merge_clusters)
        return row

    if x_col == "Time":
        times = [vp.get("timeSec") for vp in viewpoints if vp.get("timeSec") is not None]
        if not times:
            return pd.DataFrame(columns=["Time"] + metrics)
        rows = []
        for t in range(0, int(times[-1]) + 1, max(1, x_step)):
            vp = _viewpoint_at_time(viewpoints, float(t))
            if vp is not None:
                rows.append(_compute_row(vp, float(t)))
    else:
        rows = [
            _compute_row(vp, vp.get(JSON_KEY_MAP["Viewpoint"]))
            for i, vp in enumerate(viewpoints)
            if i % max(1, x_step) == 0
        ]

    out = pd.DataFrame(rows)
    if x_col not in out.columns:
        raise ValueError(f"{json_path} does not contain x column {x_col}")
    return keep_requested_columns(out, x_col, metrics)


def keep_requested_columns(df: pd.DataFrame, x_col: str, metrics: list[str]) -> pd.DataFrame:
    keep_cols = [x_col]
    for metric in metrics:
        if metric in df.columns:
            keep_cols.append(metric)
    for metric in sorted((GLOBAL_METRICS | COUNT_METRICS | DERIVED_METRICS) - set(metrics)):
        if metric in df.columns and metric in {"TP_Clusters", "FP_Clusters", "FN_Clusters"}:
            keep_cols.append(metric)
    keep_cols = list(dict.fromkeys(keep_cols))
    return df[keep_cols].sort_values(x_col)


def load_run(
    metrics_path: Path,
    classes: Optional[list[int]],
    x_col: str,
    metrics: list[str],
    distance_threshold: float,
    map_thresholds: list[float],
    f1_confidence: Optional[float] = None,
    split_clusters: bool = True,
    merge_clusters: bool = True,
    x_step: int = 1,
) -> pd.DataFrame:
    if metrics_path.name != JSON_METRICS_FILENAME and metrics_path.suffix.lower() != ".json":
        raise ValueError(f"Expected {JSON_METRICS_FILENAME}, got {metrics_path}")
    return load_json_run(metrics_path, classes, x_col, metrics, distance_threshold, map_thresholds, f1_confidence, split_clusters, merge_clusters, x_step)


def load_raw_viewpoints(json_path: Path) -> list[dict]:
    """Return the raw viewpoints array from an evaluation_metrics.json file."""
    with json_path.open("r", encoding="utf-8") as f:
        return json.load(f).get("viewpoints", [])


def make_pooled_viewpoint(viewpoint_pairs: list[tuple[str, dict]]) -> dict:
    """Merge viewpoints from multiple series into one virtual viewpoint.

    Each series gets a unique label offset so integer predicted-cluster labels
    (and matching pairwiseDistances entries) never collide across series.
    GT IDs are namespaced by series name for the same reason.
    All existing AP/F1 code (`compute_ap_for_class`, `best_distance_pairs`,
    `match_predictions_at_threshold`) works on the result without modification.
    """
    pooled_gt: list[dict] = []
    pooled_clusters: list[dict] = []
    pooled_pairwise: list[dict] = []
    for series_idx, (series_name, vp) in enumerate(viewpoint_pairs):
        offset = series_idx * 100_000
        for gt in vp.get("groundTruthSegments", []):
            pooled_gt.append({**gt, "id": f"{series_name}/{gt['id']}"})
        for pred in vp.get("predictedClusters", []):
            pooled_clusters.append({**pred, "label": pred["label"] + offset})
        for pair in vp.get("pairwiseDistances", []):
            pooled_pairwise.append({
                **pair,
                "gtId": f"{series_name}/{pair['gtId']}",
                "predictedLabel": pair["predictedLabel"] + offset,
            })
    return {
        "groundTruthSegments": pooled_gt,
        "predictedClusters": pooled_clusters,
        "pairwiseDistances": pooled_pairwise,
    }


def compute_pooled_metric_curve(
    named_sequences: list[tuple[str, list[dict]]],
    classes: Optional[list[int]],
    x_col: str,
    metrics: list[str],
    distance_threshold: float,
    map_thresholds: list[float],
    f1_confidence: Optional[float] = None,
    split_clusters: bool = True,
    merge_clusters: bool = True,
    x_step: int = 1,
) -> pd.DataFrame:
    """Compute dataset-level metrics across all series, pooled at each sample point.

    In viewpoint mode (x_col != "Time"), sample points are viewpoint indices
    0, x_step, 2*x_step, …. At each index, all series that have reached that
    viewpoint are included.

    In time mode (x_col == "Time"), sample points are integer seconds
    0, x_step, 2*x_step, … up to the latest timeSec across all series. At each
    second, each series contributes its most-recent viewpoint via zero-order hold
    (the last viewpoint whose timeSec <= t).

    In both modes the contributing viewpoints are merged into a single virtual
    viewpoint using `make_pooled_viewpoint`, then all requested metrics are
    computed on it. mAP and F1/Precision/Recall use the pooled predictions so
    that larger trees contribute proportionally (COCO-style dataset-level AP).
    Voxel-based metrics (Coverage_Percent, Occupied_Voxels, Free_Voxels) are
    averaged across contributing series since each tree has its own octomap bbox.
    """
    if not named_sequences:
        return pd.DataFrame(columns=[x_col] + metrics)

    def _pool_and_compute(pairs: list[tuple[str, dict]], x_val) -> dict:
        pooled_vp = make_pooled_viewpoint(pairs)
        row: dict = {x_col: x_val}
        need_counts = any(
            m in metrics
            for m in ("Precision", "Recall", "F1_Score", "TP_Clusters", "FP_Clusters", "FN_Clusters")
        )
        if need_counts:
            tp, fp, fn, precision, recall, f1 = compute_best_f1(
                pooled_vp, classes, distance_threshold, fixed_confidence=f1_confidence,
                split_clusters=split_clusters, merge_clusters=merge_clusters,
            )
            row.update({"TP_Clusters": tp, "FP_Clusters": fp, "FN_Clusters": fn,
                        "Precision": precision, "Recall": recall, "F1_Score": f1})
        if "mAP" in metrics:
            row["mAP"] = compute_map(pooled_vp, classes, map_thresholds,
                                     split_clusters=split_clusters, merge_clusters=merge_clusters)
        voxel_map = {
            "Coverage_Percent": "bboxCoverage",
            "Occupied_Voxels": "occupiedVoxels",
            "Free_Voxels": "freeVoxels",
        }
        for col, key in voxel_map.items():
            if col in metrics:
                vals = [vp.get(key) for _, vp in pairs if vp.get(key) is not None]
                row[col] = float(np.mean(vals)) if vals else float("nan")
        if "Total_Voxels" in metrics and "Occupied_Voxels" in row and "Free_Voxels" in row:
            row["Total_Voxels"] = row["Occupied_Voxels"] + row["Free_Voxels"]
        return row

    rows = []

    step = max(1, x_step)
    if x_col == "Time":
        all_times = [
            vp.get("timeSec")
            for _, vps in named_sequences
            for vp in vps
            if vp.get("timeSec") is not None
        ]
        if not all_times:
            return pd.DataFrame(columns=[x_col] + metrics)
        for t in range(0, int(max(all_times)) + 1, step):
            pairs = [
                (name, _viewpoint_at_time(vps, float(t)))
                for name, vps in named_sequences
            ]
            pairs = [(name, vp) for name, vp in pairs if vp is not None]
            if pairs:
                rows.append(_pool_and_compute(pairs, float(t)))
    else:
        max_len = max(len(vps) for _, vps in named_sequences)
        for v_idx in range(0, max_len, step):
            pairs = [
                (name, vps[v_idx])
                for name, vps in named_sequences
                if v_idx < len(vps)
            ]
            if not pairs:
                continue
            x_val = pairs[0][1].get("viewpointIndex", v_idx)
            rows.append(_pool_and_compute(pairs, x_val))

    df = pd.DataFrame(rows)
    ordered = [x_col] + [m for m in metrics if m in df.columns]
    return df[list(dict.fromkeys(ordered))].sort_values(x_col).reset_index(drop=True)




def output_stem(output_path: str) -> str:
    return str(Path(output_path).with_suffix(""))


def plotdata_csv_path(output_path: str) -> str:
    return output_stem(output_path) + "_plotdata.csv"


def sanitize_x_for_filename(x_value: float) -> str:
    return str(int(x_value)) if float(x_value).is_integer() else str(x_value).replace(".", "p").replace("-", "m")


def table_csv_path(output_path: str, x_value: float) -> str:
    path = Path(output_path)
    if path.suffix.lower() == ".csv":
        return str(path)
    return output_stem(output_path) + f"_table_x_{sanitize_x_for_filename(x_value)}.csv"


def table_png_path(output_path: str, x_value: float) -> str:
    path = Path(output_path)
    if path.suffix.lower() in {".png", ".jpg", ".jpeg", ".pdf", ".svg"}:
        return str(path)
    return output_stem(output_path) + f"_table_x_{sanitize_x_for_filename(x_value)}.png"


def append_plotdata_rows(
    rows: list[dict],
    series_name: str,
    metric: str,
    x: np.ndarray,
    y: np.ndarray,
    y_std: Optional[np.ndarray] = None,
):
    if y_std is None:
        y_std = np.full_like(y, np.nan, dtype=float)
    for xi, yi, si in zip(x.tolist(), y.tolist(), y_std.tolist()):
        rows.append({"series": series_name, "metric": metric, "x": xi, "y": yi, "y_std": si})


def extract_row_at_x(df: pd.DataFrame, x_col: str, x_value: float) -> Optional[pd.Series]:
    if x_col not in df.columns or df.empty:
        return None
    matches = df[df[x_col] == x_value]
    if len(matches) > 0:
        return matches.iloc[0]
    numeric_x = pd.to_numeric(df[x_col], errors="coerce")
    if numeric_x.isna().all():
        return None
    idx = (numeric_x - float(x_value)).abs().idxmin()
    return df.loc[idx] if not pd.isna(idx) else None


def build_table_dataframe(
    run_data: dict[str, pd.DataFrame],
    x_col: str,
    metrics: list[str],
    x_value: float,
) -> pd.DataFrame:
    rows = []
    for name, df in run_data.items():
        row = extract_row_at_x(df, x_col, x_value)
        if row is None:
            continue
        rows.append({"Series": name, x_col: row[x_col], **{metric: row.get(metric, np.nan) for metric in metrics}})
    return pd.DataFrame(rows)


def save_table_outputs(
    table_df: pd.DataFrame,
    output_path: str,
    x_col: str,
    x_value: float,
    title: Optional[str] = None,
    table_format: str = "csv",
):
    if table_df.empty:
        print(f"[warn] No table rows available for {x_col}={x_value}; table not written.")
        return

    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    if table_format in {"csv", "both"}:
        csv_out = table_csv_path(output_path, x_value)
        table_df.to_csv(csv_out, index=False)
        print(f"Saved table CSV to: {csv_out}")

    if table_format not in {"png", "both"}:
        return

    plt = get_pyplot()
    png_out = table_png_path(output_path, x_value)
    display_df = table_df.copy()
    for col in display_df.columns:
        if col == "Series":
            continue
        display_df[col] = display_df[col].apply(
            lambda v: "" if pd.isna(v) else f"{v:.4f}" if isinstance(v, (float, np.floating)) else str(v)
        )

    fig_w = max(10, 2.0 + 1.6 * display_df.shape[1])
    fig_h = max(2.5, 1.2 + 0.45 * display_df.shape[0])
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.axis("off")

    caption = f"Metrics table at {x_col} = {x_value}"
    if title:
        caption = f"{title}: {caption}"

    table = ax.table(cellText=display_df.values, colLabels=display_df.columns, cellLoc="center", loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.3)
    fig.suptitle(caption, fontsize=12, fontweight="bold")
    plt.tight_layout()
    plt.savefig(png_out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved table PNG to: {png_out}")


def plot_metrics(
    run_data: dict[str, pd.DataFrame],
    x_col: str,
    metrics: list[str],
    output: Optional[str],
    title: Optional[str],
    xlim: Optional[list[float]],
    ylims: list[Optional[list[float]]],
    size: str = "small",
):
    plt = get_pyplot()
    if size == "large":
        font_sizes = {"title": 22, "label": 18, "legend": 15, "ticks": 15, "suptitle": 26}
        fig_width = 14
        fig_height_per_metric = 6
        line_width = 3.0
        marker_size = 7
    else:
        font_sizes = {"title": 13, "label": 11, "legend": 8, "ticks": 10, "suptitle": 14}
        fig_width = 10
        fig_height_per_metric = 4
        line_width = 1.5
        marker_size = 4

    plt.rcParams.update({
        "axes.titlesize": font_sizes["title"],
        "axes.labelsize": font_sizes["label"],
        "xtick.labelsize": font_sizes["ticks"],
        "ytick.labelsize": font_sizes["ticks"],
        "legend.fontsize": font_sizes["legend"],
        "figure.titlesize": font_sizes["suptitle"],
    })

    n_metrics = len(metrics)
    per_metric_h = fig_height_per_metric if n_metrics == 1 else int(fig_height_per_metric * 0.75)
    fig, axes = plt.subplots(n_metrics, 1, figsize=(fig_width, per_metric_h * n_metrics), sharex=True)
    if n_metrics == 1:
        axes = [axes]

    n_series = max(1, len(run_data))
    _cmaps = [plt.cm.tab20, plt.cm.tab20b, plt.cm.tab20c]
    palette = [c for cm in _cmaps for c in (cm(i / (cm.N - 1)) for i in range(cm.N))]
    if len(palette) < n_series:
        existing = len(palette)
        palette += [plt.cm.hsv(i / (n_series - existing)) for i in range(n_series - existing)]

    sorted_run_data = dict(sorted(run_data.items()))
    plotdata_rows = []
    for i, (ax, metric) in enumerate(zip(axes, metrics)):
        color_iter = iter(palette)
        for name, df in sorted_run_data.items():
            color = next(color_iter)
            if metric not in df.columns:
                continue
            x = df[x_col].to_numpy()
            y = df[metric].to_numpy()
            ax.plot(x, y, label=name, color=color, linewidth=line_width, marker="o", markersize=marker_size)
            if output:
                append_plotdata_rows(plotdata_rows, name, metric, x, y)

        ax.set_title(metric.replace("_", " "))
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=font_sizes["legend"])
        if xlim:
            ax.set_xlim(xlim[0], xlim[1])
        ylim = ylims[i] if i < len(ylims) else None
        if ylim:
            ax.set_ylim(ylim[0], ylim[1])

    axes[-1].set_xlabel(x_col.replace("_", " "))
    if title:
        fig.suptitle(title, fontsize=font_sizes["suptitle"], fontweight="bold")
    top = 0.97 if title else 1.0
    plt.tight_layout(rect=[0, 0, 1, top])

    if output:
        out_dir = os.path.dirname(os.path.abspath(output))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        plt.savefig(output, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved plot to: {output}")
        if plotdata_rows:
            csv_out = plotdata_csv_path(output)
            pd.DataFrame(plotdata_rows).to_csv(csv_out, index=False)
            print(f"Saved plot data to: {csv_out}")
    else:
        plt.show()


def validate_metrics(metrics: list[str], parser: argparse.ArgumentParser):
    unknown = sorted(set(metrics) - set(AVAILABLE_METRICS))
    if unknown:
        parser.error(f"Unknown metric(s): {', '.join(unknown)}. Available: {', '.join(AVAILABLE_METRICS)}")


def load_run_collection(
    run_paths: dict[str, Path],
    classes: Optional[list[int]],
    x_col: str,
    metrics: list[str],
    distance_threshold: float,
    map_thresholds: list[float],
    f1_confidence: Optional[float] = None,
    split_clusters: bool = True,
    merge_clusters: bool = True,
    x_step: int = 1,
) -> dict[str, pd.DataFrame]:
    out = {}
    for run_name, metrics_path in run_paths.items():
        try:
            out[run_name] = load_run(metrics_path, classes, x_col, metrics, distance_threshold, map_thresholds, f1_confidence, split_clusters, merge_clusters, x_step)
        except Exception as exc:
            print(f"[error] Failed to load {metrics_path}: {exc}")
    return out


def load_raw_viewpoints_collection(
    run_paths: dict[str, Path],
) -> list[tuple[str, list[dict]]]:
    """Load raw viewpoint sequences for a set of named runs, skipping failures."""
    out = []
    for run_name, metrics_path in run_paths.items():
        try:
            out.append((run_name, load_raw_viewpoints(metrics_path)))
        except Exception as exc:
            print(f"[error] Failed to load {metrics_path}: {exc}")
    return out


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Plot or tabulate NBV metrics from evaluation_metrics.json files. "
            "Use --output-kind to choose exactly one output mode."
        )
    )
    parser.add_argument("--size", choices=["small", "large"], default="small",
                        help="Plot canvas and font size.")

    parser.add_argument("--runs", nargs="+", metavar="PATH",
                        help="Run directories or evaluation_metrics.json files. Each path becomes a separate series.")
    parser.add_argument("--study", metavar="DIR",
                        help="Study directory whose immediate subdirs are planners; runs are pooled per planner.")
    parser.add_argument("--metrics-root", metavar="DIR",
                        help="Metrics root containing tree dirs, each with planner dirs; all matching runs are pooled per planner.")
    parser.add_argument("--planners", nargs="+", default=None, metavar="PLANNER",
                        help="Only include these planner directory names.")
    parser.add_argument("--trees", nargs="+", default=None, metavar="TREE",
                        help="Only include these tree directory names for --metrics-root.")

    parser.add_argument("--x", choices=["time", "viewpoint", "run"], default="viewpoint",
                        help="X-axis/table lookup coordinate. 'run' is treated as viewpoint index.")
    parser.add_argument("--x-step", type=int, default=1, metavar="N",
                        help="Sample every N seconds (--x time) or every N viewpoints (--x viewpoint). Default 1 (every point).")
    parser.add_argument("--classes", nargs="+", type=int, default=None, metavar="CLASS_ID",
                        help="Restrict matching metrics to these class IDs.")
    parser.add_argument("--metrics", nargs="+", default=["mAP", "F1_Score", "Coverage_Percent"], metavar="METRIC",
                        help=f"Metrics to output. Available: {', '.join(AVAILABLE_METRICS)}.")

    parser.add_argument("--output-kind", choices=["plot", "table"], required=True,
                        help="Select plot generation or table export.")
    parser.add_argument("--output", default=None,
                        help="Output path. Plot mode writes an image and plotdata CSV; table mode writes CSV, PNG, or both.")
    parser.add_argument("--table", type=float, default=None, metavar="XVALUE",
                        help="X value to extract for --output-kind table.")
    parser.add_argument("--table-format", choices=["csv", "png", "both"], default="csv",
                        help="Table output format.")

    parser.add_argument("--distance-threshold", type=float, default=0.10,
                        help="3D distance cutoff in meters for Precision/Recall/F1 one-to-one matching.")
    parser.add_argument("--f1-confidence", type=float, default=None,
                        help="Use a fixed minimum detection confidence for F1 matching. By default, report the maximum F1 over all confidence thresholds.")
    parser.add_argument("--map-thresholds", nargs="+", type=float, default=None,
                        help="Explicit mAP distance thresholds in meters; overrides --map-threshold-{max,min,step}.")
    parser.add_argument("--map-threshold-max", type=float, default=DEFAULT_MAP_DISTANCE_THRESHOLD_MAX_M,
                        help="Largest mAP distance threshold in meters when --map-thresholds is not set.")
    parser.add_argument("--map-threshold-min", type=float, default=DEFAULT_MAP_DISTANCE_THRESHOLD_MIN_M,
                        help="Smallest mAP distance threshold in meters when --map-thresholds is not set.")
    parser.add_argument("--map-threshold-step", type=float, default=DEFAULT_MAP_DISTANCE_THRESHOLD_STEP_M,
                        help="Step size for descending mAP distance thresholds when --map-thresholds is not set.")

    parser.add_argument("--disable-cluster-splitting", action="store_true", default=False,
                        help="Do not credit matched clusters for additional nearby unmatched GTs.")
    parser.add_argument("--disable-cluster-merging", action="store_true", default=False,
                        help="Do not absorb redundant predicted clusters that are near an already-matched GT.")

    parser.add_argument("--title", default=None, help="Optional plot title or table caption prefix.")
    parser.add_argument("--xlim", nargs=2, type=float, metavar=("XMIN", "XMAX"), default=None,
                        help="Plot x-axis limits.")
    parser.add_argument("--ylim", nargs="+", type=float, metavar="VAL", default=None,
                        help="Y-axis limits per metric: YMIN1 YMAX1 [YMIN2 YMAX2 ...]. Metrics without a pair get auto-scaling.")

    args = parser.parse_args()
    validate_metrics(args.metrics, parser)

    if not args.runs and not args.study and not args.metrics_root:
        parser.error("Provide at least one of --runs, --study, or --metrics-root.")
    if args.output_kind == "table" and args.table is None:
        parser.error("--output-kind table requires --table XVALUE.")
    if args.output_kind == "table" and not args.output:
        parser.error("--output-kind table requires --output.")

    x_col = X_COLUMN_MAP[args.x]
    map_thresholds = distance_thresholds_from_args(args)

    pool_kwargs = dict(
        classes=args.classes,
        x_col=x_col,
        metrics=args.metrics,
        distance_threshold=args.distance_threshold,
        map_thresholds=map_thresholds,
        f1_confidence=args.f1_confidence,
        split_clusters=not args.disable_cluster_splitting,
        merge_clusters=not args.disable_cluster_merging,
        x_step=args.x_step,
    )

    run_data: dict[str, pd.DataFrame] = {}

    # --runs: each explicit path becomes its own series without pooling.
    if args.runs:
        run_paths = {}
        for run_arg in args.runs:
            path = Path(run_arg)
            metrics_file = path if path.is_file() else find_json_metrics_file(path)
            if not metrics_file:
                print(f"[warn] No {JSON_METRICS_FILENAME} found in {path}, skipping.")
                continue
            run_paths[path.parent.name if path.is_file() else path.name] = metrics_file
        run_data.update(load_run_collection(
            run_paths, args.classes, x_col, args.metrics,
            args.distance_threshold, map_thresholds, args.f1_confidence,
            not args.disable_cluster_splitting,
            not args.disable_cluster_merging,
            args.x_step,
        ))

    # --study: pool all runs for each planner into one series per planner.
    if args.study:
        allowed_planners = set(args.planners) if args.planners else None
        for planner_name, runs in discover_single_study(Path(args.study)).items():
            if allowed_planners and planner_name not in allowed_planners:
                continue
            sequences = load_raw_viewpoints_collection(runs)
            if not sequences:
                continue
            df = compute_pooled_metric_curve(sequences, **pool_kwargs)
            if not df.empty:
                run_data[planner_name] = df

    # --metrics-root: pool all runs across matching trees into one series per planner.
    if args.metrics_root:
        allowed_trees = set(args.trees) if args.trees else None
        allowed_planners = set(args.planners) if args.planners else None
        planner_sequences: dict[str, list[tuple[str, list[dict]]]] = {}
        for tree_name, planners in discover_metrics_root(Path(args.metrics_root)).items():
            if allowed_trees and tree_name not in allowed_trees:
                continue
            for planner_name, runs in planners.items():
                if allowed_planners and planner_name not in allowed_planners:
                    continue
                sequences = load_raw_viewpoints_collection(runs)
                planner_sequences.setdefault(planner_name, []).extend(
                    (f"{tree_name}/{name}", vps) for name, vps in sequences
                )
        for planner_name, sequences in planner_sequences.items():
            df = compute_pooled_metric_curve(sequences, **pool_kwargs)
            if not df.empty:
                run_data[planner_name] = df

    if not run_data:
        print("No data loaded. Exiting.")
        return

    if args.output_kind == "plot":
        raw_ylim = args.ylim or []
        per_metric_ylims = [
            [raw_ylim[i * 2], raw_ylim[i * 2 + 1]] if i * 2 + 1 < len(raw_ylim) else None
            for i in range(len(args.metrics))
        ]
        plot_metrics(
            run_data=run_data,
            x_col=x_col,
            metrics=args.metrics,
            output=args.output,
            title=args.title,
            xlim=args.xlim,
            ylims=per_metric_ylims,
            size=args.size,
        )
        return

    table_df = build_table_dataframe(
        run_data=run_data,
        x_col=x_col,
        metrics=args.metrics,
        x_value=args.table,
    )
    save_table_outputs(
        table_df=table_df,
        output_path=args.output,
        x_col=x_col,
        x_value=args.table,
        title=args.title,
        table_format=args.table_format,
    )


if __name__ == "__main__":
    main()
