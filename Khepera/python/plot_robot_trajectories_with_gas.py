#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from itertools import cycle

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D


def load_source_position(source_file: Path) -> tuple[float, float]:
    values = source_file.read_text(encoding="utf-8").strip().split()
    if len(values) < 2:
        raise ValueError(f"Invalid source file format: {source_file}")
    return float(values[0]), float(values[1])


def load_robot_data(robot_log_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    position_file = robot_log_dir / "position_history.csv"
    measurement_file = robot_log_dir / "measurement_history.csv"

    positions = np.genfromtxt(position_file, delimiter=",")
    measurements = np.genfromtxt(measurement_file, delimiter=",")

    if positions.ndim == 1:
        positions = positions.reshape(1, -1)
    if measurements.ndim == 1:
        measurements = measurements.reshape(1, -1)

    if positions.shape[1] < 3:
        raise ValueError(f"Expected at least 3 columns in {position_file}, got {positions.shape[1]}")
    if measurements.shape[1] < 2:
        raise ValueError(
            f"Expected at least 2 columns in {measurement_file}, got {measurements.shape[1]}"
        )

    x = positions[:, 1]
    y = positions[:, 2]
    gas_intensity = measurements[:, 1]

    n = min(len(x), len(y), len(gas_intensity))
    return x[:n], y[:n], gas_intensity[:n]


def make_line_collection(x: np.ndarray, y: np.ndarray, value: np.ndarray, cmap: str, norm) -> LineCollection:
    points = np.column_stack([x, y])
    if len(points) < 2:
        return LineCollection([], cmap=cmap, norm=norm)

    segments = np.stack([points[:-1], points[1:]], axis=1)
    segment_values = value[:-1]

    collection = LineCollection(segments, cmap=cmap, norm=norm, linewidths=2.5)
    collection.set_array(segment_values)
    return collection


def plot_trajectories(k4_dir: Path, source_file: Path, output: Path) -> None:
    robot_dirs = sorted([entry for entry in k4_dir.iterdir() if entry.is_dir()])
    if not robot_dirs:
        raise ValueError(f"No robot log directories found under: {k4_dir}")

    robot_data: list[tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []
    all_intensities: list[np.ndarray] = []

    for robot_dir in robot_dirs:
        x, y, intensity = load_robot_data(robot_dir)
        robot_data.append((robot_dir.name, x, y, intensity))
        all_intensities.append(intensity)

    merged = np.concatenate(all_intensities)
    vmin = float(np.min(merged))
    vmax = float(np.max(merged))
    if np.isclose(vmin, vmax):
        vmax = vmin + 1e-9
    norm = plt.Normalize(vmin=vmin, vmax=vmax)

    fig, ax = plt.subplots(figsize=(10, 8))
    robot_cmaps = cycle(["Reds", "Greens", "Blues", "Purples", "Oranges", "Greys"])
    robot_legend_handles: list[Line2D] = []

    for robot_name, x, y, intensity in robot_data:
        cmap_name = next(robot_cmaps)
        cmap_obj = plt.get_cmap(cmap_name)

        lc = make_line_collection(x, y, intensity, cmap=cmap_name, norm=norm)
        ax.add_collection(lc)

        start_color = cmap_obj(norm(intensity[0]))
        end_color = cmap_obj(norm(intensity[-1]))
        ax.scatter(x[0], y[0], marker="o", s=45, c=[start_color], edgecolors="black", linewidths=0.8, zorder=3)
        ax.scatter(x[-1], y[-1], marker="s", s=55, c=[end_color], edgecolors="black", linewidths=0.8, zorder=3)
        ax.text(x[-1], y[-1], f" {robot_name}", fontsize=9, va="center")

        robot_legend_handles.append(
            Line2D([0], [0], color=cmap_obj(0.8), lw=2.5, label=f"{robot_name} ({cmap_name})")
        )

    source_x, source_y = load_source_position(source_file)
    ax.scatter(
        source_x,
        source_y,
        marker="*",
        s=260,
        c="red",
        edgecolors="black",
        linewidths=1.0,
        label="Source",
        zorder=4,
    )

    legend_source = Line2D(
        [0],
        [0],
        marker="*",
        color="w",
        markerfacecolor="red",
        markeredgecolor="black",
        markersize=12,
        label="Source",
    )
    ax.legend(handles=robot_legend_handles + [legend_source], loc="best")

    ax.set_title("Robot trajectories with robot-specific color schemes")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    ax.autoscale()
    plt.tight_layout()

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200)
    print(f"Saved plot to: {output}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot trajectories of robots and color by gas intensity from measurement_history.csv column 2."
    )
    parser.add_argument(
        "root_folder",
        type=Path,
        help="Experiment root folder path (contains `k4/` and `source_position.txt`).",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    root_folder = args.root_folder
    k4_dir = root_folder / "k4"
    source_file = root_folder / "source_position.txt"
    output = k4_dir / "trajectory_gas_intensity.png"

    if not k4_dir.exists():
        raise FileNotFoundError(f"Cannot find k4 directory: {k4_dir}")
    if not source_file.exists():
        raise FileNotFoundError(f"Cannot find source position file: {source_file}")

    plot_trajectories(k4_dir, source_file, output)


if __name__ == "__main__":
    main()
