#!/usr/bin/env python3
"""
Replot NBV metrics across multiple runs.

Usage examples:
    # Single run directory
    python3 replot_metrics.py --runs path/to/run1 path/to/run2 --x time --metrics F1_Score Coverage_Percent

    # Study directory (auto-discovers subdirs containing data/nbv_metrics.csv)
    python3 replot_metrics.py --study path/to/study --x viewpoint --classes 1 --metrics F1_Score Recall Precision

    # Multiple studies, plot each run individually
    python3 replot_metrics.py --study study1 study2 --x viewpoint --metrics F1_Score

    # Multiple studies, plot per-study average (+ std band) instead of individual runs
    python3 replot_metrics.py --study study1 study2 --avg --x viewpoint --metrics F1_Score Coverage_Percent

    # Mix: individual runs + averaged studies
    python3 replot_metrics.py --runs run1 --study study_dir --avg --x time --metrics F1_Score
"""

import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


# ─── Constants ────────────────────────────────────────────────────────────────

METRICS_FILENAME = "nbv_metrics.csv"

# Columns that are per-class (will be averaged over selected classes)
PER_CLASS_METRICS = {"TP_Clusters", "FP_Clusters", "TP_Points", "FN_Points",
                     "Precision", "Recall", "F1_Score"}

# Columns that are global (not per-class, just take first row per step)
GLOBAL_METRICS = {"Coverage_Percent", "Occupied_Voxels", "Free_Voxels", "Total_Voxels"}

X_COLUMN_MAP = {
    "time":      "Time",
    "viewpoint": "Viewpoint",
    "run":       "Viewpoint",
}


# ─── Discovery ────────────────────────────────────────────────────────────────

def find_csv(path: Path) -> Path | None:
    """Given a run root, find the metrics CSV."""
    candidates = [
        path / "data" / METRICS_FILENAME,
        path / METRICS_FILENAME,
    ]
    for c in candidates:
        if c.exists():
            return c
    return None


def discover_runs(study_dir: Path) -> dict[str, Path]:
    """Return {run_name: csv_path} for all subdirs of a study directory."""
    runs = {}
    for child in sorted(study_dir.iterdir()):
        if child.is_dir():
            csv = find_csv(child)
            if csv:
                runs[child.name] = csv
    return runs


# ─── Loading ──────────────────────────────────────────────────────────────────

def load_run(csv_path: Path, classes: list[int] | None,
             x_col: str, metrics: list[str]) -> pd.DataFrame:
    """
    Load a single run CSV and return a tidy DataFrame with one row per
    viewpoint step, with per-class metrics averaged over the requested classes.

    Returns columns: [x_col] + metrics
    """
    df = pd.read_csv(csv_path)

    # Filter to requested classes (only relevant for per-class metrics)
    if classes is not None:
        df_class = df[df["Class_ID"].isin(classes)]
    else:
        df_class = df

    # Separate global vs per-class metrics requested
    requested_per_class = [m for m in metrics if m in PER_CLASS_METRICS]
    requested_global    = [m for m in metrics if m in GLOBAL_METRICS]
    unknown             = [m for m in metrics if m not in PER_CLASS_METRICS and m not in GLOBAL_METRICS]
    if unknown:
        print(f"  [warn] Unknown metric(s) ignored: {unknown}")

    rows = []
    for step, group in df.groupby(x_col, sort=True):
        row = {x_col: step}

        # Global metrics — same for all classes, just take first row
        global_group = df[df[x_col] == step]
        for m in requested_global:
            row[m] = global_group[m].iloc[0]

        # Per-class metrics — average over selected classes
        class_group = df_class[df_class[x_col] == step]
        for m in requested_per_class:
            row[m] = class_group[m].mean() if len(class_group) > 0 else float("nan")

        rows.append(row)

    return pd.DataFrame(rows)


def average_study(run_dfs: list[pd.DataFrame], x_col: str,
                  metrics: list[str]) -> tuple[pd.DataFrame, pd.DataFrame]:
    """
    Given a list of per-run DataFrames for one study, compute mean and std
    across runs at each x value (union of all x values, NaN where missing).

    Returns (mean_df, std_df), each with columns [x_col] + metrics.
    """
    # Align all runs on the union of x values
    all_x = sorted(set().union(*[set(df[x_col].tolist()) for df in run_dfs]))
    aligned = []
    for df in run_dfs:
        df_indexed = df.set_index(x_col).reindex(all_x)
        aligned.append(df_indexed)

    combined = pd.concat(aligned, axis=0, keys=range(len(aligned)))

    mean_df = combined.groupby(level=1).mean().reset_index().rename(columns={"index": x_col})
    std_df  = combined.groupby(level=1).std().reset_index().rename(columns={"index": x_col})

    mean_df[x_col] = all_x
    std_df[x_col]  = all_x

    return mean_df, std_df


# ─── Plotting ─────────────────────────────────────────────────────────────────

def plot_metrics(
    run_data: dict[str, pd.DataFrame],                          # individual runs (always plotted)
    study_data: dict[str, list[tuple[str, pd.DataFrame]]],     # study_name → [(run_name, df)]
    x_col: str,
    metrics: list[str],
    avg_studies: bool,
    output: str | None,
    title: str | None,
):
    n_metrics = len(metrics)
    fig, axes = plt.subplots(n_metrics, 1,
                             figsize=(10, 4 * n_metrics),
                             sharex=True, squeeze=False)

    # Fixed high-contrast palette — colors are perceptually distinct regardless of how
    # many series are shown. Cycles if there are more than 8 series.
    PALETTE = [
        "#e6194b",  # red
        "#3cb44b",  # green
        "#4363d8",  # blue
        "#f58231",  # orange
        "#911eb4",  # purple
        "#42d4f4",  # cyan
        "#f032e6",  # magenta
        "#bfef45",  # lime
        "#fabed4",  # pink
        "#469990",  # teal
    ]

    n_study_series = sum(len(v) for v in study_data.values()) if study_data else 0
    n_series = len(run_data) + (len(study_data) if avg_studies else n_study_series)

    for ax, metric in zip(axes[:, 0], metrics):
        # Fresh iterator each subplot so every subplot uses the same color assignment
        color_iter = (PALETTE[i % len(PALETTE)] for i in range(n_series))

        # ── Individual runs ──
        for run_name, df in run_data.items():
            color = next(color_iter)
            if metric not in df.columns:
                print(f"  [warn] Metric '{metric}' not found in run '{run_name}', skipping.")
                continue
            ax.plot(df[x_col].to_numpy(), df[metric].to_numpy(),
                    label=run_name, color=color, marker="o", markersize=3, linewidth=1.5)

        # ── Studies ──
        for study_name, named_dfs in study_data.items():
            color = next(color_iter)
            run_names = [n for n, _ in named_dfs]
            run_dfs   = [df for _, df in named_dfs]
            if avg_studies:
                # Plot mean line + ±1 std shaded band
                mean_df, std_df = average_study(run_dfs, x_col, metrics)
                if metric not in mean_df.columns:
                    continue
                x   = mean_df[x_col].to_numpy()
                y   = mean_df[metric].to_numpy()
                err = std_df[metric].to_numpy() if metric in std_df.columns else np.zeros_like(y)
                ax.plot(x, y, label=f"{study_name} (mean, n={len(run_dfs)})",
                        color=color, marker="o", markersize=3, linewidth=2.0)
                ax.fill_between(x, y - err, y + err, color=color, alpha=0.2)
            else:
                # Plot each run in the study individually with its own legend entry
                run_colors = [PALETTE[i % len(PALETTE)] for i in range(len(run_dfs))]
                for run_name, df, rc in zip(run_names, run_dfs, run_colors):
                    if metric not in df.columns:
                        continue
                    ax.plot(df[x_col].to_numpy(), df[metric].to_numpy(),
                            color=rc, marker="o", markersize=2, linewidth=1.2,
                            alpha=0.7, label=run_name)

        ax.set_ylabel(metric.replace("_", " "))
        ax.set_xlabel(x_col.replace("_", " "))
        ax.legend(loc="best", fontsize=7, ncol=max(1, n_series // 10))
        ax.grid(True, alpha=0.3)
        ax.set_title(metric.replace("_", " "))

    if title:
        fig.suptitle(title, fontsize=13, fontweight="bold")

    plt.tight_layout()

    if output:
        os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
        plt.savefig(output, dpi=150, bbox_inches="tight")
        print(f"Saved plot to: {output}")
    else:
        plt.show()


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Replot NBV metrics across runs.")

    # Input sources
    parser.add_argument("--runs", nargs="+", metavar="DIR",
                        help="One or more run directories (each containing data/nbv_metrics.csv).")
    parser.add_argument("--study", nargs="+", metavar="DIR",
                        help="One or more study directories whose subdirs are runs.")
    parser.add_argument("--labels", nargs="+", metavar="LABEL",
                        help="Optional display labels for --runs (must match count).")
    parser.add_argument("--study-labels", nargs="+", metavar="LABEL",
                        help="Optional display labels for --study dirs (must match count).")

    # X axis
    parser.add_argument("--x", choices=["time", "viewpoint", "run"],
                        default="viewpoint",
                        help="X axis: 'time' (seconds) or 'viewpoint'/'run' (step index). Default: viewpoint")

    # Classes
    parser.add_argument("--classes", nargs="+", type=int, default=None,
                        metavar="CLASS_ID",
                        help="Class IDs to include for per-class metrics (averaged). "
                             "Default: all classes.")

    # Metrics
    parser.add_argument("--metrics", nargs="+",
                        default=["F1_Score", "Coverage_Percent"],
                        metavar="METRIC",
                        help="Metrics to plot. Default: F1_Score Coverage_Percent. "
                             f"Per-class options: {sorted(PER_CLASS_METRICS)}. "
                             f"Global options: {sorted(GLOBAL_METRICS)}.")

    # Averaging
    parser.add_argument("--avg", action="store_true",
                        help="For each study, plot the mean across runs with a ±1 std shaded band "
                             "instead of individual run lines.")

    # Output
    parser.add_argument("--output", default=None,
                        help="Save plot to this file path instead of showing interactively.")
    parser.add_argument("--title", default=None,
                        help="Overall plot title.")

    args = parser.parse_args()

    if not args.runs and not args.study:
        parser.error("Provide at least one of --runs or --study.")

    # ── Resolve x column ──
    x_col = X_COLUMN_MAP[args.x]

    # ── Collect individual runs ──
    individual_csvs: dict[str, Path] = {}
    if args.runs:
        labels = args.labels if args.labels else [None] * len(args.runs)
        if args.labels and len(args.labels) != len(args.runs):
            parser.error("--labels count must match --runs count.")
        for run_dir, label in zip(args.runs, labels):
            p = Path(run_dir)
            csv = find_csv(p)
            if csv is None:
                print(f"[warn] No metrics CSV found in {p}, skipping.")
                continue
            individual_csvs[label if label else p.name] = csv

    # ── Collect studies ──
    # study_csvs: study_name → {run_name: csv_path}
    study_csvs: dict[str, dict[str, Path]] = {}
    if args.study:
        study_labels = args.study_labels if args.study_labels else [None] * len(args.study)
        if args.study_labels and len(args.study_labels) != len(args.study):
            parser.error("--study-labels count must match --study count.")
        for study_dir, slabel in zip(args.study, study_labels):
            p = Path(study_dir)
            discovered = discover_runs(p)
            if not discovered:
                print(f"[warn] No runs found under study dir {p}.")
                continue
            name = slabel if slabel else p.name
            study_csvs[name] = discovered
            print(f"Study '{name}': {len(discovered)} run(s)")
            for rn, rcsv in discovered.items():
                print(f"  {rn:50s} -> {rcsv}")

    if not individual_csvs and not study_csvs:
        print("No valid runs found. Exiting.")
        return

    # ── Load individual runs ──
    run_data: dict[str, pd.DataFrame] = {}
    for name, csv in individual_csvs.items():
        try:
            run_data[name] = load_run(csv, args.classes, x_col, args.metrics)
        except Exception as e:
            print(f"[error] Failed to load {csv}: {e}")

    # ── Load study runs ──
    study_data: dict[str, list[tuple[str, pd.DataFrame]]] = {}
    for study_name, run_csvs in study_csvs.items():
        named_dfs = []
        for run_name, csv in run_csvs.items():
            try:
                named_dfs.append((run_name, load_run(csv, args.classes, x_col, args.metrics)))
            except Exception as e:
                print(f"[error] Failed to load {csv}: {e}")
        if named_dfs:
            study_data[study_name] = named_dfs

    if not run_data and not study_data:
        print("No data loaded. Exiting.")
        return

    # ── Plot ──
    plot_metrics(run_data, study_data, x_col, args.metrics, args.avg, args.output, args.title)


if __name__ == "__main__":
    main()