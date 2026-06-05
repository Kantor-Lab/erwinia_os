#!/usr/bin/env python3
"""
Plot or tabulate NBV evaluation metrics from the current evaluation_metrics.json
format.

Common examples:
  # Plot one JSON run.
  python3 replot_metrics.py --runs metrics/penn_state/tree_11/semantic --output-kind plot \
      --metrics mAP F1_Score Coverage_Percent --output semantic.png

  # Table at viewpoint 20 for one tree/study with planner directories.
  python3 replot_metrics.py --study metrics/penn_state/tree_11 --avg --output-kind table \
      --metrics mAP F1_Score Coverage_Percent --table 20 --output tree_11_summary.csv

  # Plot planner means across trees.
  python3 replot_metrics.py --metrics-root metrics/penn_state --avg-trees --output-kind plot \
      --metrics mAP F1_Score Coverage_Percent --output planner_means.png

JSON metric definitions:
  F1_Score is the maximum F1 achieved across all confidence thresholds.
  Predictions are processed in descending confidence order; at each implicit
  threshold, predictions are matched one-to-one to the nearest unmatched
  same-class GT within --distance-threshold meters, and the TP/FP/FN yielding
  the highest F1 are reported.

  mAP is computed like detection AP, but the "IoU threshold" is replaced by
  a 3D distance threshold in meters. By default AP is averaged over thresholds
  0.30, 0.27, ..., 0.03 and over classes that have ground truth. At each
  threshold, detections are sorted by confidence and matched one-to-one to the
  nearest unmatched same-class ground-truth segment within that distance.
"""

import argparse
import json
import os
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd


JSON_METRICS_FILENAME = "evaluation_metrics.json"

DEFAULT_MAP_DISTANCE_THRESHOLD_MAX_M = 0.30
DEFAULT_MAP_DISTANCE_THRESHOLD_MIN_M = 0.03
DEFAULT_MAP_DISTANCE_THRESHOLD_STEP_M = 0.03

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

    tp = len(used_gt)
    fp = len(predictions) - len(used_pred)
    fn = len(gt_ids) - len(used_gt)
    return tp, fp, fn



def compute_ap_for_class(viewpoint: dict, class_id: int, distance_threshold: float) -> float:
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
        # Predictions with no GT are all FP → AP = 0; no predictions → don't count this class
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
        else:
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


def compute_map(viewpoint: dict, classes: Optional[list[int]], distance_thresholds: list[float]) -> float:
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
            ap = compute_ap_for_class(viewpoint, class_id, threshold)
            if not pd.isna(ap):
                ap_values.append(ap)
    return float(np.mean(ap_values)) if ap_values else float("nan")


def recompute_derived_metrics_from_counts(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    if {"TP_Clusters", "FP_Clusters", "FN_Clusters"}.issubset(out.columns):
        values = out.apply(
            lambda row: compute_precision_recall_f1(row["TP_Clusters"], row["FP_Clusters"], row["FN_Clusters"]),
            axis=1,
            result_type="expand",
        )
        out["Precision"] = values[0]
        out["Recall"] = values[1]
        out["F1_Score"] = values[2]
    return out


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


def load_json_run(
    json_path: Path,
    classes: Optional[list[int]],
    x_col: str,
    metrics: list[str],
    distance_threshold: float,
    map_thresholds: list[float],
    f1_confidence: float = 0.0,
) -> pd.DataFrame:
    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    viewpoints = data.get("viewpoints", [])

    rows = []
    for viewpoint in viewpoints:
        row = {
            "Viewpoint": viewpoint.get(JSON_KEY_MAP["Viewpoint"]),
            "Time": viewpoint.get(JSON_KEY_MAP["Time"]),
            "Coverage_Percent": viewpoint.get(JSON_KEY_MAP["Coverage_Percent"]),
            "Occupied_Voxels": viewpoint.get(JSON_KEY_MAP["Occupied_Voxels"]),
            "Free_Voxels": viewpoint.get(JSON_KEY_MAP["Free_Voxels"]),
        }
        if row["Occupied_Voxels"] is not None and row["Free_Voxels"] is not None:
            row["Total_Voxels"] = row["Occupied_Voxels"] + row["Free_Voxels"]

        if any(metric in metrics for metric in ("Precision", "Recall", "F1_Score", "TP_Clusters", "FP_Clusters", "FN_Clusters")):
            tp, fp, fn = match_predictions_at_threshold(viewpoint, classes, distance_threshold, min_confidence=f1_confidence)
            precision, recall, f1 = compute_precision_recall_f1(tp, fp, fn)
            row.update({
                "TP_Clusters": tp,
                "FP_Clusters": fp,
                "FN_Clusters": fn,
                "Precision": precision,
                "Recall": recall,
                "F1_Score": f1,
            })

        if "mAP" in metrics:
            row["mAP"] = compute_map(viewpoint, classes, map_thresholds)

        rows.append(row)

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
    f1_confidence: float = 0.0,
) -> pd.DataFrame:
    if metrics_path.name != JSON_METRICS_FILENAME and metrics_path.suffix.lower() != ".json":
        raise ValueError(f"Expected {JSON_METRICS_FILENAME}, got {metrics_path}")
    return load_json_run(metrics_path, classes, x_col, metrics, distance_threshold, map_thresholds, f1_confidence)


def average_dfs_at_x(dfs: list[pd.DataFrame], x_col: str, metrics: list[str]) -> tuple[pd.DataFrame, pd.DataFrame]:
    if not dfs:
        empty = pd.DataFrame(columns=[x_col] + metrics)
        return empty, empty

    all_x = sorted(set().union(*[set(df[x_col].tolist()) for df in dfs if x_col in df.columns]))
    if not all_x:
        empty = pd.DataFrame(columns=[x_col] + metrics)
        return empty, empty

    aligned = [df.set_index(x_col).reindex(all_x) for df in dfs]
    combined = pd.concat(aligned, axis=0, keys=range(len(aligned)))
    mean_df = combined.groupby(level=1).mean(numeric_only=True).reset_index().rename(columns={"index": x_col})
    std_df = combined.groupby(level=1).std(numeric_only=True).reset_index().rename(columns={"index": x_col})
    mean_df[x_col] = all_x
    std_df[x_col] = all_x

    present_count_cols = [col for col in COUNT_METRICS if col in combined.columns]
    if present_count_cols:
        summed_counts = combined.groupby(level=1)[present_count_cols].sum(min_count=1).reset_index().rename(
            columns={"index": x_col}
        )
        for col in present_count_cols:
            mean_df[col] = summed_counts[col].to_numpy()
        mean_df = recompute_derived_metrics_from_counts(mean_df)

    ordered_cols = [x_col] + [metric for metric in metrics if metric in mean_df.columns]
    std_cols = [x_col] + [metric for metric in metrics if metric in std_df.columns]
    return mean_df[ordered_cols], std_df[std_cols]


def compute_planner_mean_std_over_runs(
    planner_run_dfs: dict[str, list[pd.DataFrame]],
    x_col: str,
    metrics: list[str],
) -> dict[str, tuple[pd.DataFrame, pd.DataFrame]]:
    return {
        planner: average_dfs_at_x(run_dfs, x_col, metrics)
        for planner, run_dfs in planner_run_dfs.items()
    }


def average_tree_then_across_trees(
    tree_planner_run_dfs: dict[str, dict[str, list[pd.DataFrame]]],
    x_col: str,
    metrics: list[str],
) -> dict[str, tuple[pd.DataFrame, pd.DataFrame]]:
    planner_tree_means = {}
    for _tree, planners in tree_planner_run_dfs.items():
        for planner, run_dfs in planners.items():
            tree_mean_df, _ = average_dfs_at_x(run_dfs, x_col, metrics)
            planner_tree_means.setdefault(planner, []).append(tree_mean_df)
    return {
        planner: average_dfs_at_x(tree_mean_dfs, x_col, metrics)
        for planner, tree_mean_dfs in planner_tree_means.items()
    }


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
    planner_meanstd_runs: dict[str, tuple[pd.DataFrame, pd.DataFrame]],
    planner_meanstd_trees: dict[str, tuple[pd.DataFrame, pd.DataFrame]],
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

    for planner, (mean_df, _std_df) in planner_meanstd_runs.items():
        row = extract_row_at_x(mean_df, x_col, x_value)
        if row is None:
            continue
        rows.append({
            "Series": f"{planner} (mean over runs)",
            x_col: row[x_col],
            **{metric: row.get(metric, np.nan) for metric in metrics},
        })

    for planner, (mean_df, _std_df) in planner_meanstd_trees.items():
        row = extract_row_at_x(mean_df, x_col, x_value)
        if row is None:
            continue
        rows.append({
            "Series": f"{planner} (mean over trees)",
            x_col: row[x_col],
            **{metric: row.get(metric, np.nan) for metric in metrics},
        })

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
    planner_meanstd_runs: dict[str, tuple[pd.DataFrame, pd.DataFrame]],
    planner_meanstd_trees: dict[str, tuple[pd.DataFrame, pd.DataFrame]],
    x_col: str,
    metrics: list[str],
    output: Optional[str],
    title: Optional[str],
    xlim: Optional[list[float]],
    ylim: Optional[list[float]],
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

    fig, axes = plt.subplots(len(metrics), 1, figsize=(fig_width, fig_height_per_metric * len(metrics)), sharex=True)
    if len(metrics) == 1:
        axes = [axes]

    plotdata_rows = []
    n_series = max(1, len(run_data) + len(planner_meanstd_runs) + len(planner_meanstd_trees))

    # Build a palette large enough that colors never repeat.
    # Sample tab20 → tab20b → tab20c (60 perceptually distinct colors total),
    # then fill any remainder with evenly-spaced HSV hues.
    _cmaps = [plt.cm.tab20, plt.cm.tab20b, plt.cm.tab20c]
    palette = [c for cm in _cmaps for c in (cm(i / (cm.N - 1)) for i in range(cm.N))]
    if len(palette) < n_series:
        existing = len(palette)
        palette += [plt.cm.hsv(i / (n_series - existing)) for i in range(n_series - existing)]

    for ax, metric in zip(axes, metrics):
        color_iter = iter(palette)

        for name, df in run_data.items():
            color = next(color_iter)
            if metric not in df.columns:
                continue
            x = df[x_col].to_numpy()
            y = df[metric].to_numpy()
            ax.plot(x, y, label=name, color=color, linewidth=line_width, marker="o", markersize=marker_size)
            if output:
                append_plotdata_rows(plotdata_rows, name, metric, x, y)

        for planner, (mean_df, std_df) in planner_meanstd_runs.items():
            color = next(color_iter)
            if metric not in mean_df.columns:
                continue
            x = mean_df[x_col].to_numpy()
            y = mean_df[metric].to_numpy()
            err = std_df[metric].to_numpy() if metric in std_df.columns else np.zeros_like(y)
            label = f"{planner} (mean +/- std over runs)"
            ax.plot(x, y, label=label, color=color, linewidth=line_width, marker="o", markersize=marker_size)
            ax.fill_between(x, y - err, y + err, color=color, alpha=0.2)
            if output:
                append_plotdata_rows(plotdata_rows, label, metric, x, y, err)

        for planner, (mean_df, std_df) in planner_meanstd_trees.items():
            color = next(color_iter)
            if metric not in mean_df.columns:
                continue
            x = mean_df[x_col].to_numpy()
            y = mean_df[metric].to_numpy()
            err = std_df[metric].to_numpy() if metric in std_df.columns else np.zeros_like(y)
            label = f"{planner} (mean +/- std over trees)"
            ax.plot(x, y, label=label, color=color, linewidth=line_width, marker="o", markersize=marker_size)
            ax.fill_between(x, y - err, y + err, color=color, alpha=0.2)
            if output:
                append_plotdata_rows(plotdata_rows, label, metric, x, y, err)

        ax.set_ylabel(metric.replace("_", " "))
        ax.set_title(metric.replace("_", " "))
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=font_sizes["legend"])
        if xlim:
            ax.set_xlim(xlim[0], xlim[1])
        if ylim:
            ax.set_ylim(ylim[0], ylim[1])

    axes[-1].set_xlabel(x_col.replace("_", " "))
    if title:
        fig.suptitle(title, fontsize=font_sizes["suptitle"], fontweight="bold")
    plt.tight_layout()

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
    f1_confidence: float = 0.0,
) -> dict[str, pd.DataFrame]:
    out = {}
    for run_name, metrics_path in run_paths.items():
        try:
            out[run_name] = load_run(metrics_path, classes, x_col, metrics, distance_threshold, map_thresholds, f1_confidence)
        except Exception as exc:
            print(f"[error] Failed to load {metrics_path}: {exc}")
    return out


def main():
    parser = argparse.ArgumentParser(description="Plot or tabulate NBV metrics from evaluation_metrics.json files.")
    parser.add_argument("--size", choices=["small", "large"], default="small", help="Plot and font size.")

    parser.add_argument("--runs", nargs="+", metavar="DIR", help="One or more run directories or evaluation_metrics.json files.")
    parser.add_argument("--study", metavar="DIR", help="Directory whose immediate subdirs are planner dirs.")
    parser.add_argument("--metrics-root", metavar="DIR", help="Root containing tree dirs, each with planner dirs.")
    parser.add_argument("--planners", nargs="+", default=None, metavar="PLANNER", help="Restrict planners.")
    parser.add_argument("--trees", nargs="+", default=None, metavar="TREE", help="Restrict trees for --metrics-root.")

    parser.add_argument("--x", choices=["time", "viewpoint", "run"], default="viewpoint")
    parser.add_argument("--classes", nargs="+", type=int, default=None, metavar="CLASS_ID")
    parser.add_argument("--metrics", nargs="+", default=["mAP", "F1_Score", "Coverage_Percent"], metavar="METRIC")

    parser.add_argument("--output-kind", choices=["plot", "table"], required=True, help="Choose exactly what to write/show.")
    parser.add_argument("--output", default=None, help="Output file. Plot: image path. Table: CSV or PNG path.")
    parser.add_argument("--table", type=float, default=None, metavar="XVALUE", help="X-axis value for --output-kind table.")
    parser.add_argument("--table-format", choices=["csv", "png", "both"], default="csv")

    parser.add_argument("--avg", action="store_true", help="For --study: aggregate per planner across runs.")
    parser.add_argument("--avg-trees", action="store_true", help="For --metrics-root: aggregate runs per tree, then trees per planner.")

    parser.add_argument("--distance-threshold", type=float, default=0.10,
                        help="3D distance threshold in meters used as the IoU-equivalent cutoff for Precision/Recall/F1 matching.")
    parser.add_argument("--f1-confidence", type=float, default=0.0,
                        help="Minimum detection confidence for F1 matching (default 0.0 = no filtering).")
    parser.add_argument("--map-thresholds", nargs="+", type=float, default=None,
                        help="Explicit mAP distance thresholds in meters.")
    parser.add_argument("--map-threshold-max", type=float, default=DEFAULT_MAP_DISTANCE_THRESHOLD_MAX_M)
    parser.add_argument("--map-threshold-min", type=float, default=DEFAULT_MAP_DISTANCE_THRESHOLD_MIN_M)
    parser.add_argument("--map-threshold-step", type=float, default=DEFAULT_MAP_DISTANCE_THRESHOLD_STEP_M)

    parser.add_argument("--title", default=None)
    parser.add_argument("--xlim", nargs=2, type=float, metavar=("XMIN", "XMAX"), default=None)
    parser.add_argument("--ylim", nargs=2, type=float, metavar=("YMIN", "YMAX"), default=None)

    args = parser.parse_args()
    validate_metrics(args.metrics, parser)

    if not args.runs and not args.study and not args.metrics_root:
        parser.error("Provide at least one of --runs, --study, or --metrics-root.")
    if args.avg and not args.study:
        parser.error("--avg requires --study.")
    if args.avg_trees and not args.metrics_root:
        parser.error("--avg-trees requires --metrics-root.")
    if args.output_kind == "table" and args.table is None:
        parser.error("--output-kind table requires --table XVALUE.")
    if args.output_kind == "table" and not args.output:
        parser.error("--output-kind table requires --output.")

    x_col = X_COLUMN_MAP[args.x]
    map_thresholds = distance_thresholds_from_args(args)

    run_data = {}
    if args.runs:
        run_paths = {}
        for run_arg in args.runs:
            path = Path(run_arg)
            metrics_file = path if path.is_file() else find_json_metrics_file(path)
            if not metrics_file:
                print(f"[warn] No {JSON_METRICS_FILENAME} found in {path}, skipping.")
                continue
            run_paths[path.parent.name if path.is_file() else path.name] = metrics_file
        run_data = load_run_collection(
            run_paths, args.classes, x_col, args.metrics, args.distance_threshold, map_thresholds, args.f1_confidence
        )

    planner_meanstd_runs = {}
    if args.study:
        planners = discover_single_study(Path(args.study))
        allowed_planners = set(args.planners) if args.planners else None
        planner_run_dfs = {}
        for planner_name, runs in planners.items():
            if allowed_planners and planner_name not in allowed_planners:
                continue
            loaded = load_run_collection(
                runs, args.classes, x_col, args.metrics, args.distance_threshold, map_thresholds, args.f1_confidence
            )
            if loaded:
                planner_run_dfs[planner_name] = list(loaded.values())
        if args.avg:
            planner_meanstd_runs = compute_planner_mean_std_over_runs(planner_run_dfs, x_col, args.metrics)
        else:
            for planner_name, dfs in planner_run_dfs.items():
                for i, df in enumerate(dfs, start=1):
                    run_data[f"{planner_name}/run_{i}"] = df

    planner_meanstd_trees = {}
    if args.metrics_root and args.avg_trees:
        discovered = discover_metrics_root(Path(args.metrics_root))
        allowed_trees = set(args.trees) if args.trees else None
        allowed_planners = set(args.planners) if args.planners else None
        tree_planner_run_dfs = {}
        for tree_name, planners in discovered.items():
            if allowed_trees and tree_name not in allowed_trees:
                continue
            tree_planner_run_dfs[tree_name] = {}
            for planner_name, runs in planners.items():
                if allowed_planners and planner_name not in allowed_planners:
                    continue
                loaded = load_run_collection(
                    runs, args.classes, x_col, args.metrics, args.distance_threshold, map_thresholds, args.f1_confidence
                )
                if loaded:
                    tree_planner_run_dfs[tree_name][planner_name] = list(loaded.values())
        planner_meanstd_trees = average_tree_then_across_trees(tree_planner_run_dfs, x_col, args.metrics)

    if not run_data and not planner_meanstd_runs and not planner_meanstd_trees:
        print("No data loaded. Exiting.")
        return

    if args.output_kind == "plot":
        plot_metrics(
            run_data=run_data,
            planner_meanstd_runs=planner_meanstd_runs,
            planner_meanstd_trees=planner_meanstd_trees,
            x_col=x_col,
            metrics=args.metrics,
            output=args.output,
            title=args.title,
            xlim=args.xlim,
            ylim=args.ylim,
            size=args.size,
        )
        return

    table_df = build_table_dataframe(
        run_data=run_data,
        planner_meanstd_runs=planner_meanstd_runs,
        planner_meanstd_trees=planner_meanstd_trees,
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
