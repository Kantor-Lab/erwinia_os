#!/usr/bin/env python3
"""
Replot NBV metrics across runs, single studies, multi-studies, and planner-over-trees experiments.

Supports:
  1) Individual run directories via --runs
  2) A single study directory via --study (immediate subdirs are planner dirs)
     study/
       baseline/
         run_1/
         ...
       semantic/
         run_1/
         ...
       volumetric/
         run_1/
         ...
     With --avg-trees enabled, this will compute per-planner mean±std across
     the "tree dimension" — but here the study has only ONE "tree", so std across trees is 0.
     Typically for a single study you want mean±std across runs per planner, use --avg.
  3) Multi-tree metrics-root via --metrics-root:
     metrics/
       tree_1/
         baseline/run_*/...
       tree_2/...
     With --avg-trees enabled: average runs within each (tree, planner), then mean±std across trees per planner.

Plot data export:
  If --output is set, also writes <output_basename>_plotdata.csv
  with columns: series, metric, x, y, y_std

Axis bounds:
  --xlim xmin xmax
  --ylim ymin ymax

Examples:
  # Single study, plot per-planner mean±std across runs:
  python3 replot_metrics.py --study path/to/study --avg --x viewpoint --metrics F1_Score Coverage_Percent

  # Metrics root, plot per-planner mean±std across trees (tree means computed from runs):
  python3 replot_metrics.py --metrics-root metrics --avg-trees --x viewpoint --metrics F1_Score
"""

import argparse
import os
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

METRICS_FILENAME = "nbv_metrics.csv"

PER_CLASS_METRICS = {
    "TP_Clusters", "FP_Clusters", "TP_Points", "FN_Points",
    "Precision", "Recall", "F1_Score"
}

GLOBAL_METRICS = {
    "Coverage_Percent", "Occupied_Voxels", "Free_Voxels", "Total_Voxels"
}

X_COLUMN_MAP = {
    "time": "Time",
    "viewpoint": "Viewpoint",
    "run": "Viewpoint",
}


# ──────────────────────────────────────────────────────────────────────────────
# Discovery
# ──────────────────────────────────────────────────────────────────────────────

def find_csv(path: Path) -> Optional[Path]:
    candidates = [
        path / "data" / METRICS_FILENAME,
        path / METRICS_FILENAME,
    ]
    for c in candidates:
        if c.exists():
            return c
    return None


def discover_runs(container_dir: Path) -> dict[str, Path]:
    """Return {run_name: csv_path} for all subdirs that are runs."""
    runs: dict[str, Path] = {}
    for child in sorted(container_dir.iterdir()):
        if child.is_dir():
            csv = find_csv(child)
            if csv:
                runs[child.name] = csv
    return runs


def discover_metrics_root(metrics_root: Path) -> dict[str, dict[str, dict[str, Path]]]:
    """
    metrics_root/
      tree_X/
        planner_Y/
          run_Z/
            (data/nbv_metrics.csv)
    Returns:
      tree -> planner -> run -> csv_path
    """
    out: dict[str, dict[str, dict[str, Path]]] = {}
    for tree_dir in sorted(metrics_root.iterdir()):
        if not tree_dir.is_dir():
            continue
        planners: dict[str, dict[str, Path]] = {}
        for planner_dir in sorted(tree_dir.iterdir()):
            if not planner_dir.is_dir():
                continue
            runs = discover_runs(planner_dir)
            if runs:
                planners[planner_dir.name] = runs
        if planners:
            out[tree_dir.name] = planners
    return out


def discover_single_study(study_dir: Path) -> dict[str, dict[str, Path]]:
    """
    study_dir/
      baseline/
        run_1/
      semantic/
      volumetric/
    Returns:
      planner -> run -> csv_path
    """
    planners: dict[str, dict[str, Path]] = {}
    for planner_dir in sorted(study_dir.iterdir()):
        if not planner_dir.is_dir():
            continue
        runs = discover_runs(planner_dir)
        if runs:
            planners[planner_dir.name] = runs
    return planners


# ──────────────────────────────────────────────────────────────────────────────
# Loading / Aggregation
# ──────────────────────────────────────────────────────────────────────────────

def load_run(csv_path: Path, classes: Optional[list[int]], x_col: str, metrics: list[str]) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    if classes is not None:
        df_class = df[df["Class_ID"].isin(classes)]
    else:
        df_class = df

    rows = []

    for step, _ in df.groupby(x_col, sort=True):
        row = {x_col: step}

        global_group = df[df[x_col] == step]
        for m in metrics:
            if m in GLOBAL_METRICS and m in global_group.columns:
                row[m] = global_group[m].iloc[0] if len(global_group) else float("nan")
            elif m in PER_CLASS_METRICS and m in df_class.columns:
                class_group = df_class[df_class[x_col] == step]
                row[m] = class_group[m].mean() if len(class_group) else float("nan")

        rows.append(row)

    return pd.DataFrame(rows)


def average_dfs_at_x(dfs: list[pd.DataFrame], x_col: str, metrics: list[str]) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Align on union of x, then compute mean and std across dfs."""
    if not dfs:
        empty = pd.DataFrame(columns=[x_col] + metrics)
        return empty, empty

    all_x = sorted(set().union(*[set(df[x_col].tolist()) for df in dfs if x_col in df.columns]))
    if not all_x:
        empty = pd.DataFrame(columns=[x_col] + metrics)
        return empty, empty

    aligned = [df.set_index(x_col).reindex(all_x) for df in dfs]
    combined = pd.concat(aligned, axis=0, keys=range(len(aligned)))

    mean_df = combined.groupby(level=1).mean().reset_index().rename(columns={"index": x_col})
    std_df  = combined.groupby(level=1).std().reset_index().rename(columns={"index": x_col})

    mean_df[x_col] = all_x
    std_df[x_col]  = all_x

    return mean_df, std_df


def compute_planner_mean_std_over_runs(
    planner_run_dfs: dict[str, list[pd.DataFrame]],
    x_col: str,
    metrics: list[str],
) -> dict[str, tuple[pd.DataFrame, pd.DataFrame]]:
    """
    For a SINGLE study:
      planner -> list[run_df]
    Return:
      planner -> (mean_df, std_df) across runs
    """
    out: dict[str, tuple[pd.DataFrame, pd.DataFrame]] = {}
    for planner, run_dfs in planner_run_dfs.items():
        mean_df, std_df = average_dfs_at_x(run_dfs, x_col, metrics)
        out[planner] = (mean_df, std_df)
    return out


def average_tree_then_across_trees(
    tree_planner_run_dfs: dict[str, dict[str, list[pd.DataFrame]]],
    x_col: str,
    metrics: list[str],
) -> dict[str, tuple[pd.DataFrame, pd.DataFrame]]:
    """
    For metrics-root:
      - per (tree, planner): average over runs -> tree_mean_df
      - per planner: mean±std across trees of tree_mean_df
    Return:
      planner -> (mean_df, std_df) across trees
    """
    planner_tree_means: dict[str, list[pd.DataFrame]] = {}

    for _tree, planners in tree_planner_run_dfs.items():
        for planner, run_dfs in planners.items():
            tree_mean_df, _ = average_dfs_at_x(run_dfs, x_col, metrics)
            planner_tree_means.setdefault(planner, []).append(tree_mean_df)

    out: dict[str, tuple[pd.DataFrame, pd.DataFrame]] = {}
    for planner, tree_mean_dfs in planner_tree_means.items():
        mean_df, std_df = average_dfs_at_x(tree_mean_dfs, x_col, metrics)
        out[planner] = (mean_df, std_df)

    return out


# ──────────────────────────────────────────────────────────────────────────────
# Plot data export helpers
# ──────────────────────────────────────────────────────────────────────────────

def plotdata_csv_path(output_path: str) -> str:
    p = Path(output_path)
    return str(p.with_suffix("")) + "_plotdata.csv"


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
        rows.append({
            "series": series_name,
            "metric": metric,
            "x": xi,
            "y": yi,
            "y_std": si,
        })


# ──────────────────────────────────────────────────────────────────────────────
# Plotting
# ──────────────────────────────────────────────────────────────────────────────

def plot_metrics(
    run_data: dict[str, pd.DataFrame],  # raw individual runs (optional)
    planner_meanstd_runs: dict[str, tuple[pd.DataFrame, pd.DataFrame]],   # for --study + --avg
    planner_meanstd_trees: dict[str, tuple[pd.DataFrame, pd.DataFrame]],  # for --metrics-root + --avg-trees
    x_col: str,
    metrics: list[str],
    output: Optional[str],
    title: Optional[str],
    xlim: Optional[list[float]],
    ylim: Optional[list[float]],
    size: str = "small",
):
    # Font/plot size settings
    if size == "large":
        font_sizes = {
            "title": 22,
            "label": 18,
            "legend": 15,
            "ticks": 15,
            "suptitle": 26,
        }
        fig_width = 14
        fig_height_per_metric = 6
        line_widths = {"run": 2.5, "mean_run": 3.5, "mean_tree": 4.0}
        marker_size = 7
    else:
        font_sizes = {
            "title": 13,
            "label": 11,
            "legend": 8,
            "ticks": 10,
            "suptitle": 14,
        }
        fig_width = 10
        fig_height_per_metric = 4
        line_widths = {"run": 1.2, "mean_run": 2.0, "mean_tree": 2.2}
        marker_size = 3

    plt.rcParams.update({
        "axes.titlesize": font_sizes["title"],
        "axes.labelsize": font_sizes["label"],
        "xtick.labelsize": font_sizes["ticks"],
        "ytick.labelsize": font_sizes["ticks"],
        "legend.fontsize": font_sizes["legend"],
        "figure.titlesize": font_sizes["suptitle"],
    })

    n_metrics = len(metrics)
    fig, axes = plt.subplots(n_metrics, 1, figsize=(fig_width, fig_height_per_metric * n_metrics), sharex=True)
    if n_metrics == 1:
        axes = [axes]

    PALETTE = [
        "#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
        "#42d4f4", "#f032e6", "#bfef45", "#fabed4", "#469990",
    ]

    # Plot-data export
    plotdata_rows: list[dict] = []

    # Count series for palette cycling
    n_series = max(
        1,
        len(run_data) + len(planner_meanstd_runs) + len(planner_meanstd_trees)
    )

    for ax, metric in zip(axes, metrics):
        color_iter = (PALETTE[i % len(PALETTE)] for i in range(n_series))

        # Raw runs (if any)
        for name, df in run_data.items():
            color = next(color_iter)
            if metric not in df.columns:
                continue
            x = df[x_col].to_numpy()
            y = df[metric].to_numpy()
            ax.plot(x, y, label=name, color=color, linewidth=line_widths["run"], marker="o", markersize=marker_size)

            if output:
                append_plotdata_rows(plotdata_rows, name, metric, x, y)

        # Single study planners (mean±std across runs)
        for planner, (mean_df, std_df) in planner_meanstd_runs.items():
            color = next(color_iter)
            if metric not in mean_df.columns:
                continue
            x = mean_df[x_col].to_numpy()
            y = mean_df[metric].to_numpy()
            err = std_df[metric].to_numpy() if metric in std_df.columns else np.zeros_like(y)

            label = f"{planner} (mean±std over runs)"
            ax.plot(x, y, label=label, color=color, linewidth=line_widths["mean_run"], marker="o", markersize=marker_size+1)
            ax.fill_between(x, y - err, y + err, color=color, alpha=0.2)

            if output:
                append_plotdata_rows(plotdata_rows, label, metric, x, y, err)

        # Metrics-root planners (mean±std across trees)
        for planner, (mean_df, std_df) in planner_meanstd_trees.items():
            color = next(color_iter)
            if metric not in mean_df.columns:
                continue
            x = mean_df[x_col].to_numpy()
            y = mean_df[metric].to_numpy()
            err = std_df[metric].to_numpy() if metric in std_df.columns else np.zeros_like(y)

            label = f"{planner} (mean±std over trees)"
            ax.plot(x, y, label=label, color=color, linewidth=line_widths["mean_tree"], marker="o", markersize=marker_size+2)
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
        print(f"Saved plot to: {output}")

        if plotdata_rows:
            df_plot = pd.DataFrame(plotdata_rows)
            csv_out = plotdata_csv_path(output)
            df_plot.to_csv(csv_out, index=False)
            print(f"Saved plot data to: {csv_out}")
        else:
            print("[warn] No plot data collected; plot-data CSV not written.")
    else:
        plt.show()


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Replot NBV metrics across runs/studies/trees.")
    parser.add_argument("--size", choices=["small", "large"], default="small", help="Plot and font size: small (default) or large.")

    # Inputs
    parser.add_argument("--runs", nargs="+", metavar="DIR",
                        help="One or more run directories (each containing data/nbv_metrics.csv).")
    parser.add_argument("--study", metavar="DIR",
                        help="A single study directory whose immediate subdirs are planner dirs.")
    parser.add_argument("--metrics-root", metavar="DIR",
                        help="Root directory containing tree dirs (each containing planner dirs).")

    parser.add_argument("--planners", nargs="+", default=None, metavar="PLANNER",
                        help="Restrict planners (for --study or --metrics-root).")
    parser.add_argument("--trees", nargs="+", default=None, metavar="TREE",
                        help="Restrict trees (for --metrics-root).")

    # X axis
    parser.add_argument("--x", choices=["time", "viewpoint", "run"],
                        default="viewpoint")

    # Classes / Metrics
    parser.add_argument("--classes", nargs="+", type=int, default=None, metavar="CLASS_ID")
    parser.add_argument("--metrics", nargs="+", default=["F1_Score", "Coverage_Percent"], metavar="METRIC")

    # Averaging modes
    parser.add_argument("--avg", action="store_true",
                        help="For --study: plot per-planner mean±std across runs.")
    parser.add_argument("--avg-trees", action="store_true",
                        help="For --metrics-root: plot per-planner mean±std across trees (tree means computed from runs).")

    # Output / title
    parser.add_argument("--output", default=None)
    parser.add_argument("--title", default=None)

    # Axis bounds
    parser.add_argument("--xlim", nargs=2, type=float, metavar=("XMIN", "XMAX"), default=None)
    parser.add_argument("--ylim", nargs=2, type=float, metavar=("YMIN", "YMAX"), default=None)

    args = parser.parse_args()

    if not args.runs and not args.study and not args.metrics_root:
        parser.error("Provide at least one of --runs, --study, or --metrics-root.")

    if args.avg and not args.study:
        parser.error("--avg is intended for --study (single study mean±std over runs).")

    if args.avg_trees and not args.metrics_root:
        parser.error("--avg-trees requires --metrics-root.")

    x_col = X_COLUMN_MAP[args.x]

    # ── Load raw runs (optional) ──
    run_data: dict[str, pd.DataFrame] = {}
    if args.runs:
        for run_dir in args.runs:
            p = Path(run_dir)
            csv = find_csv(p)
            if not csv:
                print(f"[warn] No metrics CSV found in {p}, skipping.")
                continue
            try:
                run_data[p.name] = load_run(csv, args.classes, x_col, args.metrics)
            except Exception as e:
                print(f"[error] Failed to load {csv}: {e}")

    # ── Single study aggregation (mean±std across runs per planner) ──
    planner_meanstd_runs: dict[str, tuple[pd.DataFrame, pd.DataFrame]] = {}
    if args.study:
        study_dir = Path(args.study)
        planners = discover_single_study(study_dir)
        if not planners:
            print(f"[warn] No planners/runs found under study dir {study_dir}")
        else:
            allowed_planners = set(args.planners) if args.planners else None
            planner_run_dfs: dict[str, list[pd.DataFrame]] = {}

            for planner_name, runs in planners.items():
                if allowed_planners and planner_name not in allowed_planners:
                    continue
                dfs: list[pd.DataFrame] = []
                for _run_name, csv in runs.items():
                    try:
                        dfs.append(load_run(csv, args.classes, x_col, args.metrics))
                    except Exception as e:
                        print(f"[error] Failed to load {csv}: {e}")
                if dfs:
                    planner_run_dfs[planner_name] = dfs

            if args.avg:
                planner_meanstd_runs = compute_planner_mean_std_over_runs(planner_run_dfs, x_col, args.metrics)
            else:
                # If not averaging, you can still see raw runs by passing them via --runs,
                # but we won't auto-plot each planner run here to avoid legend explosion.
                print("[info] --study provided without --avg; nothing aggregated from study. "
                      "Use --avg to plot per-planner mean±std across runs.")

    # ── Metrics-root aggregation (mean±std across trees per planner) ──
    planner_meanstd_trees: dict[str, tuple[pd.DataFrame, pd.DataFrame]] = {}
    if args.metrics_root and args.avg_trees:
        root = Path(args.metrics_root)
        discovered = discover_metrics_root(root)
        if not discovered:
            print(f"[warn] No trees/planners/runs found under metrics root {root}")
        else:
            allowed_trees = set(args.trees) if args.trees else None
            allowed_planners = set(args.planners) if args.planners else None

            tree_planner_run_dfs: dict[str, dict[str, list[pd.DataFrame]]] = {}
            for tree_name, planners in discovered.items():
                if allowed_trees and tree_name not in allowed_trees:
                    continue
                tree_planner_run_dfs[tree_name] = {}
                for planner_name, runs in planners.items():
                    if allowed_planners and planner_name not in allowed_planners:
                        continue
                    dfs: list[pd.DataFrame] = []
                    for _run_name, csv in runs.items():
                        try:
                            dfs.append(load_run(csv, args.classes, x_col, args.metrics))
                        except Exception as e:
                            print(f"[error] Failed to load {csv}: {e}")
                    if dfs:
                        tree_planner_run_dfs[tree_name][planner_name] = dfs

            planner_meanstd_trees = average_tree_then_across_trees(tree_planner_run_dfs, x_col, args.metrics)

    if not run_data and not planner_meanstd_runs and not planner_meanstd_trees:
        print("No data loaded. Exiting.")
        return

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


if __name__ == "__main__":
    main()