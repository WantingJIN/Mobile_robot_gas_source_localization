#!/usr/bin/env python3
"""
compare_results.py

Analyze GSL (Gas Source Localization) experiment results for both single-robot
and multi-robot cases.

Edit GENERAL_FOLDER and DISPATCHED_FOLDER below, then just run:
    python3 compare_results.py

The script auto-detects whether there are undispatched experiments in
GENERAL_FOLDER and dispatches them first; otherwise it goes straight to
analysis of the already-dispatched DISPATCHED_FOLDER.
"""

import argparse
import os
import re
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")  # change to "Agg" for headless / no-display environments
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ===========================================================================
# DEFAULT PATHS  –  edit these two lines to match your setup
# ===========================================================================
GENERAL_FOLDER    = "/home/wjin/Workspace/Multi-robot_GSL/Results"
DISPATCHED_FOLDER = "/home/wjin/Workspace/Multi-robot_GSL/Results/test/single_robot_relative_value"
# Folder names inside GENERAL_FOLDER that are never treated as experiments
DISPATCH_EXCLUDED = {".", "..", "current", "101", "102", "103"}
# ===========================================================================


# ---------------------------------------------------------------------------
# param_log.csv parser
# ---------------------------------------------------------------------------

def parse_param_log(filepath):
    """Parse space-separated key-value param_log.csv → dict of {key: np.ndarray}."""
    params = {}
    source_pos_fallback = Path(filepath).parent / "source_pos.csv"
    files_to_read = [filepath]
    if not _has_source_position(filepath) and source_pos_fallback.is_file():
        files_to_read.append(str(source_pos_fallback))

    for f in files_to_read:
        with open(f, "r") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                key = parts[0]
                values = []
                for v in parts[1:]:
                    try:
                        values.append(float(v))
                    except ValueError:
                        pass
                if values:
                    params[key] = np.array(values)
    return params


def _has_source_position(filepath):
    with open(filepath, "r") as f:
        return any(line.startswith("source_position") for line in f)


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def read_csv_drop_trailing(filepath):
    """Read CSV with possible trailing comma → drop all-NaN columns."""
    if not os.path.isfile(filepath):
        return None
    df = pd.read_csv(filepath, header=None)
    return df.dropna(axis=1, how="all")


def read_experiment_time(filepath):
    """Return 'total time (s)' value from experiment_time.csv, or NaN."""
    if not os.path.isfile(filepath):
        return np.nan
    with open(filepath, "r") as f:
        lines = f.readlines()
    for line in reversed(lines):
        m = re.search(r"total time \(s\):\s*([\d.]+)", line)
        if m:
            return float(m.group(1))
    return np.nan


# ---------------------------------------------------------------------------
# Metric helpers
# ---------------------------------------------------------------------------

def traj_length_from_xy(xy: np.ndarray) -> float:
    if xy is None or len(xy) < 2:
        return np.nan
    diffs = np.diff(xy, axis=0)
    return float(np.sum(np.sqrt(np.sum(diffs ** 2, axis=1))))


def compute_traj_length_single(robot_folder: str) -> float:
    """Single-robot: from trajectory.csv columns [iter, x, y, ...]."""
    df = read_csv_drop_trailing(os.path.join(robot_folder, "trajectory.csv"))
    if df is None or len(df) < 2:
        return np.nan
    return traj_length_from_xy(df.iloc[:, 1:3].values.astype(float))


def compute_traj_length_multi(robot_folder: str) -> float:
    """Multi-robot: from whole_position_history.csv columns [x, y, ...]."""
    df = read_csv_drop_trailing(os.path.join(robot_folder, "whole_position_history.csv"))
    if df is None or len(df) < 2:
        return 0.0  # missing robot just contributes 0
    return traj_length_from_xy(df.iloc[:, 0:2].values.astype(float))


def compute_balance_scores(robot_folders):
    """Compute normalised H_geometric and H_informative across all robot folders."""
    H_geo_vals, H_info_vals = [], []

    for folder in robot_folders:
        geo_f = os.path.join(folder, "cluster_sizes_geometric.csv")
        info_f = os.path.join(folder, "cluster_sizes_informative.csv")
        if not (os.path.isfile(geo_f) and os.path.isfile(info_f)):
            continue

        data_geo = read_csv_drop_trailing(geo_f)
        data_info = read_csv_drop_trailing(info_f)
        if data_geo is None or data_info is None:
            continue

        n_robots = data_geo.shape[1]
        log_n = np.log2(n_robots) if n_robots > 1 else 1.0

        for mat, dst in [(data_geo.values.astype(float), H_geo_vals),
                         (data_info.values.astype(float), H_info_vals)]:
            for row in mat:
                total = row.sum()
                if total > 0:
                    p = row[row > 0] / total
                    dst.append(-np.sum(p * np.log2(p)) / log_n)
                else:
                    dst.append(0.0)

    H_geo = float(np.nanmean(H_geo_vals)) if H_geo_vals else np.nan
    H_info = float(np.nanmean(H_info_vals)) if H_info_vals else np.nan
    return H_geo, H_info


# ---------------------------------------------------------------------------
# Label mappings
# ---------------------------------------------------------------------------

SAMPLING_MODES = {0: "sense_in_motion", 1: "stop_sense_go", 2: "visit_sense_go"}
PLANNING_MODES = {0: "greedy", 5: "belief_clustering", 6: "informative_kld"}
SENSOR_MODES   = {0: "PID", 1: "MOX", 2: "NOISY_MOX", 3: "SLOW_MOX", 4: "VERY_SLOW_MOX"}
PLUME_MODELS   = {0: "Gaussian", 1: "DDPM", 2: "PDE"}
GAS_FEATURES   = {0: "absolute_value", 1: "gas_hit", 2: "relative_value"}


def get_wind_config(params):
    wi  = int(params.get("wind_info",             [0])[0])
    we  = int(params.get("wind_estimation_method", [0])[0])
    wu  = int(params.get("model_uncertainty",      [0])[0])
    if wi == 0:
        return "wind_gt"
    if wi == 1 and we == 2:
        return "baseflow"
    if wi == 1 and we == 1 and wu == 0:
        return "Gaussian_fixed_uncertainty"
    if wi == 1 and we == 1 and wu == 1:
        return "Gaussian_dynamic_uncertainty"
    return "unknown"


# ---------------------------------------------------------------------------
# Step 1 – Dispatch
# ---------------------------------------------------------------------------

def dispatch_results(general_folder, dispatched_folder, excluded=None):
    """
    Move completed experiment folders from general_folder →
    dispatched_folder/<map_nbr>/<experiment_folder>.

    An experiment is considered complete when any robot subfolder contains
    position_history.csv.
    """
    if excluded is None:
        excluded = {".", "..", "current", "101", "102", "103"}

    general_folder   = Path(general_folder)
    dispatched_folder = Path(dispatched_folder)
    dispatched_folder.mkdir(parents=True, exist_ok=True)

    candidates = sorted(
        d for d in general_folder.iterdir()
        if d.is_dir() and d.name not in excluded
    )

    moved = 0
    for exp_folder in candidates:
        robot_dirs = sorted(d for d in exp_folder.iterdir() if d.is_dir())
        if not robot_dirs:
            continue

        # Use first robot subfolder for params
        ref_robot = robot_dirs[0]
        param_file = ref_robot / "param_log.csv"
        if not param_file.is_file():
            continue

        params = parse_param_log(str(param_file))

        # Skip unfinished experiments (no position_history anywhere)
        has_position = any(
            (r / "position_history.csv").is_file() for r in robot_dirs
        )
        if not has_position:
            print(f"  [skip unfinished] {exp_folder.name}")
            continue

        map_nbr = int(params.get("map_nbr", [0])[0])
        target = dispatched_folder / str(map_nbr)
        target.mkdir(parents=True, exist_ok=True)

        dest = target / exp_folder.name
        if dest.exists():
            print(f"  [already exists]  {exp_folder.name} → {map_nbr}/")
            continue

        shutil.move(str(exp_folder), str(dest))
        print(f"  [moved]           {exp_folder.name} → {map_nbr}/")
        moved += 1

    print(f"Dispatched {moved} experiment(s).\n")


# ---------------------------------------------------------------------------
# Step 2 – Extract data
# ---------------------------------------------------------------------------

def find_experiment_folders(dispatched_folder):
    """
    Walk dispatched_folder and return (exp_path, [robot_dir, ...]) for every
    experiment folder (i.e. a folder whose immediate children contain
    param_log.csv).
    """
    root = Path(dispatched_folder)
    results = []

    for dirpath, dirnames, _ in os.walk(str(root)):
        dirpath = Path(dirpath)
        robot_dirs = sorted(
            dirpath / d for d in dirnames
            if (dirpath / d / "param_log.csv").is_file()
        )
        if robot_dirs:
            results.append((dirpath, robot_dirs))

    return results


def extract_experiment_data(dispatched_folder):
    """Extract metrics from all experiments and return a DataFrame."""
    experiments = find_experiment_folders(dispatched_folder)
    print(f"Found {len(experiments)} experiment(s) under {dispatched_folder}")

    rows = []
    for exp_folder, robot_dirs in experiments:
        ref_robot = robot_dirs[0]
        params    = parse_param_log(str(ref_robot / "param_log.csv"))
        map_nbr   = int(params.get("map_nbr",           [0])[0])
        n_robots  = int(params.get("number_of_robots",  [1])[0])
        is_multi  = n_robots > 1

        # Skip if experiment appears incomplete
        if not (ref_robot / "position_history.csv").is_file():
            continue

        # --- expected_a_posteriori ---
        eap_df = read_csv_drop_trailing(str(ref_robot / "expected_a_posteriori.csv"))
        if eap_df is None:
            continue
        last_eap = eap_df.iloc[-1].values.astype(float)

        if is_multi:
            # col 0 = iteration counter, cols 1,2 = x,y
            iteration_nbr = int(last_eap[0])
            eap_xy        = last_eap[1:3]
        else:
            # row count = iterations, cols 1,2 = x,y
            iteration_nbr = len(eap_df)
            eap_xy        = last_eap[1:3]

        # --- max_a_posteriori (NaN fallback) ---
        map_df = read_csv_drop_trailing(str(ref_robot / "max_a_posteriori.csv"))
        if map_df is not None:
            map_xy = map_df.iloc[-1, 1:3].values.astype(float)
        else:
            map_xy = np.array([np.nan, np.nan])
        eap_xy = np.where(np.isnan(eap_xy), map_xy, eap_xy)

        # --- localisation error ---
        src = params.get("source_position", np.array([np.nan, np.nan]))
        error = src[:2] - eap_xy
        estimate_error = float(np.linalg.norm(error))

        # --- trajectory length ---
        if is_multi:
            traj_length = sum(
                compute_traj_length_multi(str(r)) for r in robot_dirs
            )
        else:
            traj_length = compute_traj_length_single(str(ref_robot))

        # --- experiment time ---
        exp_time = read_experiment_time(str(ref_robot / "experiment_time.csv"))

        # --- balance scores (multi-robot only) ---
        if is_multi:
            H_geo, H_info = compute_balance_scores([str(r) for r in robot_dirs])
        else:
            H_geo, H_info = np.nan, np.nan

        # --- strategy labels ---
        sampling_mode = SAMPLING_MODES.get(int(params.get("sampling_mode",    [0])[0]), "unknown")
        planning_mode = PLANNING_MODES.get(int(params.get("planning_strategy",[0])[0]), "unknown")
        sensor_mode   = SENSOR_MODES.get(  int(params.get("gas_sensor_type",  [0])[0]), "unknown")
        plume_model   = PLUME_MODELS.get(  int(params.get("plume_model",      [0])[0]), "unknown")
        gas_feature   = GAS_FEATURES.get(  int(params.get("gas_feature",      [0])[0]), "unknown")
        wind_config   = get_wind_config(params)

        rows.append({
            "Map":               map_nbr,
            "x_error":           float(error[0]),
            "y_error":           float(error[1]),
            "iteration":         iteration_nbr,
            "exp_time":          exp_time,
            "traj_length":       traj_length,
            "estimate_error":    estimate_error,
            "n_robots":          n_robots,
            "sampling_mode":     sampling_mode,
            "planning_mode":     planning_mode,
            "sensor_mode":       sensor_mode,
            "plume_model":       plume_model,
            "gas_feature":       gas_feature,
            "wind_config":       wind_config,
            "geometric_score":   H_geo,
            "informative_score": H_info,
        })

    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Step 3 – Plot
# ---------------------------------------------------------------------------

# Success criterion: |x_error| < 1 m  AND  |y_error| < 0.5 m
# The bounding-circle radius of that rectangle is sqrt(1² + 0.5²) ≈ 1.12 m,
# used as a reference line on the Euclidean-distance boxplot.
_SUCCESS_RADIUS = float(np.sqrt(1.0 ** 2 + 0.5 ** 2))  # ≈ 1.118 m


def _is_success(sub: pd.DataFrame) -> "pd.Series":
    """Return boolean Series: True when the run counts as a success."""
    return (sub["x_error"].abs() < 1.0) & (sub["y_error"].abs() < 0.5)


def plot_results(df: pd.DataFrame, dispatched_folder):
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    dispatched_folder = Path(dispatched_folder)
    maps    = sorted(df["Map"].unique())
    n_maps  = len(maps)
    is_multi = df["n_robots"].max() > 1

    blue = np.array([44, 136, 217]) / 255

    metrics = ["estimate_error", "iteration", "exp_time", "traj_length"]
    titles  = ["Localization Error (Euclidean)", "Iterations", "Experiment Time", "Traveling Distance"]
    ylabels = ["Error (m)",                      "Iterations", "Time (s)",        "Distance (m)"]
    ylims   = [(0, None),                         (0, 40),      None,              None]

    fig, axes = plt.subplots(1, len(metrics), figsize=(5 * len(metrics), 7))
    fig.patch.set_facecolor("white")

    for ax, metric, title, ylabel, ylim in zip(axes, metrics, titles, ylabels, ylims):
        groups = [df.loc[df["Map"] == m, metric].dropna().values for m in maps]

        ax.boxplot(
            groups,
            positions   = range(1, n_maps + 1),
            widths      = 0.5,
            patch_artist= True,
            medianprops = dict(color="red",   linewidth=1.5),
            boxprops    = dict(facecolor=(*blue, 0.5), color=blue),
            whiskerprops= dict(color=blue, linewidth=1.2),
            capprops    = dict(color=blue, linewidth=1.2),
            flierprops  = dict(marker="+", color="black", markersize=5),
        )
        ax.set_xlim(0.5, n_maps + 0.5)
        ax.set_xticks(range(1, n_maps + 1))
        ax.set_xticklabels([str(m) for m in maps], fontweight="bold")
        ax.set_xlabel("Map",   fontweight="bold")
        ax.set_ylabel(ylabel,  fontweight="bold")
        ax.set_title(title,    fontweight="bold")
        if ylim is not None:
            ax.set_ylim(ylim)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=10)

        if metric == "estimate_error":
            # Bounding-circle of the success rectangle (|x|<1, |y|<0.5)
            ax.axhline(
                _SUCCESS_RADIUS, color="green", linestyle="--",
                linewidth=1.2, alpha=0.85,
            )

            # Per-map pie charts overlaid on the boxplot
            # Each inset is centred above its corresponding box
            pie_w = min(0.18, 0.82 / n_maps)
            # Scale height so the inset is square in display pixels
            # (each subplot is ~5 in wide × 7 in tall → ratio ≈ 5/7)
            pie_h = pie_w * (7.0 / 5.0)

            for i, m in enumerate(maps):
                pos     = i + 1
                sub     = df.loc[df["Map"] == m].dropna(subset=["x_error", "y_error"])
                if len(sub) == 0:
                    continue
                n_suc  = int(_is_success(sub).sum())
                n_fail = len(sub) - n_suc
                rate   = n_suc / len(sub)

                # x-centre in axes-fraction coords
                x_ctr = (pos - 0.5) / n_maps
                inset = ax.inset_axes(
                    [x_ctr - pie_w / 2, 0.70, pie_w, pie_h]
                )
                inset.patch.set_alpha(0.75)

                if n_fail == 0:
                    inset.pie([1], colors=["#2ca02c"],
                              wedgeprops=dict(linewidth=0.5, edgecolor="white"))
                elif n_suc == 0:
                    inset.pie([1], colors=["#d62728"],
                              wedgeprops=dict(linewidth=0.5, edgecolor="white"))
                else:
                    inset.pie(
                        [n_suc, n_fail],
                        colors=["#2ca02c", "#d62728"],
                        startangle=90,
                        wedgeprops=dict(linewidth=0.5, edgecolor="white"),
                    )
                inset.set_title(
                    f"{rate * 100:.0f}%", fontsize=7, pad=1, fontweight="bold"
                )

            # Legend for threshold line + pie colours
            legend_handles = [
                Line2D([0], [0], color="green", linestyle="--", linewidth=1.2,
                       label=f"Bounding radius {_SUCCESS_RADIUS:.2f} m  (|x|<1, |y|<0.5)"),
                Patch(facecolor="#2ca02c", edgecolor="white", label="Success"),
                Patch(facecolor="#d62728", edgecolor="white", label="Failure"),
            ]
            ax.legend(handles=legend_handles, fontsize=7, loc="upper right")

    # --- footer annotation ---
    overall_suc  = _is_success(df.dropna(subset=["x_error", "y_error"])).sum()
    overall_rate = overall_suc / len(df)
    footer = f"Overall success rate: {overall_rate * 100:.1f}%  (|x|<1 m, |y|<0.5 m)"

    if is_multi:
        g_avg = df["geometric_score"].mean()
        i_avg = df["informative_score"].mean()
        footer += (
            f"     |     Avg Geometric Score: {g_avg * 100:.1f}%"
            f"     |     Avg Informative Score: {i_avg * 100:.1f}%"
        )

    fig.text(
        0.5, 0.01, footer,
        ha="center", va="bottom", fontsize=11, fontweight="bold",
        bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.85),
    )

    plt.tight_layout(rect=[0, 0.07, 1, 1])

    save_path = dispatched_folder / "results_boxplot.png"
    plt.savefig(str(save_path), dpi=150, bbox_inches="tight")
    print(f"Plot saved → {save_path}")
    plt.show()


# ---------------------------------------------------------------------------
# Auto-detect undispatched experiments
# ---------------------------------------------------------------------------

def has_undispatched(general_folder, dispatched_folder, excluded):
    """
    Return True if GENERAL_FOLDER contains any direct-child experiment folder
    that has not yet been moved into DISPATCHED_FOLDER.

    An 'experiment folder' is a direct subdirectory of general_folder whose
    first robot subfolder contains param_log.csv AND position_history.csv.
    """
    general_folder    = Path(general_folder)
    dispatched_folder = Path(dispatched_folder)

    for candidate in sorted(general_folder.iterdir()):
        if not candidate.is_dir():
            continue
        if candidate.name in excluded:
            continue
        # skip if this IS the dispatched folder (or a parent of it)
        try:
            candidate.resolve().relative_to(dispatched_folder.resolve())
            continue  # candidate is inside dispatched_folder
        except ValueError:
            pass
        try:
            dispatched_folder.resolve().relative_to(candidate.resolve())
            continue  # dispatched_folder is inside candidate
        except ValueError:
            pass

        robot_dirs = sorted(d for d in candidate.iterdir() if d.is_dir())
        if not robot_dirs:
            continue
        ref = robot_dirs[0]
        if (ref / "param_log.csv").is_file() and any(
            (r / "position_history.csv").is_file() for r in robot_dirs
        ):
            return True
    return False


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Compare GSL experiment results (single- and multi-robot).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--general_folder", type=str, default=GENERAL_FOLDER,
        help=f"Folder with raw experiment results (default: {GENERAL_FOLDER}).",
    )
    parser.add_argument(
        "--dispatched_folder", type=str, default=DISPATCHED_FOLDER,
        help=f"Output folder sorted by map ID (default: {DISPATCHED_FOLDER}).",
    )
    parser.add_argument(
        "--excluded", nargs="+", default=list(DISPATCH_EXCLUDED),
        help="Folder names to skip during dispatch.",
    )
    args = parser.parse_args()

    general_folder    = args.general_folder
    dispatched_folder = args.dispatched_folder
    excluded          = set(args.excluded)

    # 1. Auto-dispatch if undispatched experiments are found
    if has_undispatched(general_folder, dispatched_folder, excluded):
        print(f"=== Undispatched experiments found in: {general_folder}")
        dispatch_results(general_folder, dispatched_folder, excluded)
    else:
        print(f"=== No undispatched experiments found — using existing: {dispatched_folder}")

    # 2. Extract
    print(f"\n=== Extracting data from: {dispatched_folder}")
    df = extract_experiment_data(dispatched_folder)

    if df.empty:
        print("No complete experiments found. Exiting.")
        return

    maps = sorted(df["Map"].unique().tolist())
    print(f"\nExtracted {len(df)} rows across maps: {maps}")
    print(df[["Map", "x_error", "y_error", "iteration", "exp_time", "traj_length",
              "estimate_error", "n_robots"]].to_string(index=False))

    # 3. Save CSV
    csv_path = Path(dispatched_folder) / "experiment_data.csv"
    df.to_csv(str(csv_path), index=False)
    print(f"\nData saved → {csv_path}")

    # 4. Plot
    print("\n=== Plotting…")
    plot_results(df, dispatched_folder)


if __name__ == "__main__":
    main()
