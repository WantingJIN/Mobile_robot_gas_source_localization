#!/usr/bin/env python3
"""
Cluster and trajectory animation for multi-robot GSL experiments.

Python version of MT_Multirobot_Simulation_ClustersAndTrajectories.m.

By default this uses the same base folder as the MATLAB script and selects the
first experiment subfolder. The map number is read from 101/param_log.csv and
the corresponding obstacle list is loaded from openfoam/test_maps.

Examples:
    python MT_Multirobot_Simulation_ClustersAndTrajectories.py
    python MT_Multirobot_Simulation_ClustersAndTrajectories.py /path/to/base/10
    python MT_Multirobot_Simulation_ClustersAndTrajectories.py /path/to/experiment/1779901195967
    python MT_Multirobot_Simulation_ClustersAndTrajectories.py /path/to/experiment/1779901195967/101
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.animation as animation
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_BASE_PATH = Path(
    "/home/wjin/Workspace/Multi-robot_GSL/Results/multi_test/PDE_with_shared_wind/10"
)
DEFAULT_ROBOT_IDS = (101, 102, 103)
DEFAULT_REPR_VEL = 6
DEFAULT_VIDEO_FPS = 60
DEFAULT_VIDEO_DPI = 140

WALL_THICKNESS = 0.1
WORLD_X_LIMITS = (-WALL_THICKNESS, 16.0 + WALL_THICKNESS)
WORLD_Y_LIMITS = (-1.35, 4.0 + WALL_THICKNESS)

CELL_X_MIN = 3.0
CELL_X_MAX = 12.5
CELL_Y_MIN = 0.16
CELL_Y_MAX = 3.8

WHITE = np.array([1.0, 1.0, 1.0])
RED_SOFT = np.array([0.88, 0.35, 0.32])
BLUE_SOFT = np.array([0.30, 0.52, 0.86])
GREEN_SOFT = np.array([0.28, 0.68, 0.38])
YELLOW = np.array([0.98, 0.72, 0.14])
DARK_GRAY = np.array([0.22, 0.24, 0.27])
AX_FACE = "#f7f8fb"
GRID_COLOR = "#d9dee7"
SPINE_COLOR = "#9aa4b2"
TEXT_COLOR = "#22272e"

CLASS_COLORS = {
    0: WHITE,
    1: RED_SOFT,
    2: BLUE_SOFT,
    3: GREEN_SOFT,
}


@dataclass
class ExperimentData:
    base_path: Path
    exp_folder: Path
    robot_ids: tuple[int, ...]
    map_nbr: int
    obstacles: list[tuple[float, float, float, float]]
    positions: list[np.ndarray]
    goal_positions: list[np.ndarray]
    belief_maps: np.ndarray
    intensity_maps: np.ndarray
    clusters_geometric: np.ndarray
    clusters_informative: np.ndarray
    source_xy: tuple[float, float]
    trajectories: list[np.ndarray]


def parse_param_log(path: Path) -> dict[str, float | list[float]]:
    params: dict[str, float | list[float]] = {}
    with path.open() as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            values: list[float] = []
            for value in parts[1:]:
                try:
                    values.append(float(value))
                except ValueError:
                    pass
            if values:
                params[parts[0]] = values[0] if len(values) == 1 else values
    return params


def load_csv(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open() as f:
        for line in f:
            parts = [part.strip() for part in line.strip().split(",")]
            if not parts:
                continue
            try:
                row = [float(part) for part in parts if part != ""]
            except ValueError:
                continue
            if row:
                rows.append(row)

    if not rows:
        return np.empty((0, 0))

    ncols = max(len(row) for row in rows)
    arr = np.full((len(rows), ncols), np.nan)
    for row_idx, row in enumerate(rows):
        arr[row_idx, : len(row)] = row
    return arr


def parse_source_position(path: Path, params: dict[str, float | list[float]]) -> tuple[float, float]:
    source = params.get("source_position")
    if isinstance(source, list) and len(source) >= 2:
        return float(source[0]), float(source[1])

    if not path.exists():
        raise FileNotFoundError(f"Could not find source position at {path}")

    parts = path.read_text().strip().split()
    values = [float(part) for part in parts[1:]]
    if len(values) < 2:
        raise ValueError(f"Could not parse source position from {path}")
    return values[0], values[1]


def workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


def load_obstacles(map_nbr: int, root: Path) -> list[tuple[float, float, float, float]]:
    obstacle_path = (
        root
        / "openfoam"
        / "test_maps"
        / f"map{map_nbr}"
        / f"map{map_nbr}_wind0.75"
        / f"{map_nbr}_obstacles.txt"
    )
    obstacles: list[tuple[float, float, float, float]] = []
    if not obstacle_path.exists():
        print(f"Warning: obstacle file not found: {obstacle_path}")
        return obstacles

    with obstacle_path.open() as f:
        lines = f.readlines()

    for line in lines[1:]:
        parts = line.strip().split()
        if len(parts) < 4:
            continue
        try:
            x1, y1, x2, y2 = (float(part) for part in parts[:4])
        except ValueError:
            continue
        obstacles.append((x1, y1, x2, y2))

    return obstacles


def has_robot_folders(path: Path, robot_ids: Iterable[int]) -> bool:
    return all((path / str(robot_id)).is_dir() for robot_id in robot_ids)


def resolve_experiment_folder(
    selected_path: Path,
    robot_ids: tuple[int, ...],
    experiment_index: int,
) -> tuple[Path, Path]:
    selected_path = selected_path.expanduser().resolve()

    if (selected_path / "param_log.csv").exists() and selected_path.name in {
        str(robot_id) for robot_id in robot_ids
    }:
        exp_folder = selected_path.parent
        return exp_folder.parent, exp_folder

    if has_robot_folders(selected_path, robot_ids):
        return selected_path.parent, selected_path

    candidates = [
        child
        for child in sorted(selected_path.iterdir())
        if child.is_dir() and not child.name.startswith(".") and has_robot_folders(child, robot_ids)
    ]
    if not candidates:
        raise FileNotFoundError(f"No experiment subfolder found in {selected_path}")
    if experiment_index < 0 or experiment_index >= len(candidates):
        raise IndexError(
            f"Experiment index {experiment_index} out of range; found {len(candidates)} subfolders"
        )
    return selected_path, candidates[experiment_index]


def normalized_entropy(cluster_sizes: np.ndarray, n_clusters: int = 3) -> float:
    total = np.nansum(cluster_sizes)
    if total == 0:
        return 0.0
    probabilities = cluster_sizes / total
    probabilities = probabilities[probabilities > 0]
    if probabilities.size == 0:
        return 0.0
    return float(-np.sum(probabilities * np.log2(probabilities)) / np.log2(n_clusters))


def load_experiment(
    selected_path: Path,
    robot_ids: tuple[int, ...],
    experiment_index: int,
) -> ExperimentData:
    base_path, exp_folder = resolve_experiment_folder(selected_path, robot_ids, experiment_index)
    reference_robot = exp_folder / str(robot_ids[0])
    params_path = reference_robot / "param_log.csv"
    if not params_path.exists():
        raise FileNotFoundError(f"Missing param_log.csv: {params_path}")

    params = parse_param_log(params_path)
    if "map_nbr" not in params:
        raise KeyError(f"map_nbr not found in {params_path}")
    map_nbr = int(params["map_nbr"])
    obstacles = load_obstacles(map_nbr, workspace_root())

    positions: list[np.ndarray] = []
    goal_positions: list[np.ndarray] = []
    trajectories: list[np.ndarray] = []

    for robot_id in robot_ids:
        robot_path = exp_folder / str(robot_id)
        whole_position = load_csv(robot_path / "whole_position_history.csv")
        goal_position = load_csv(robot_path / "goal_position.csv")
        trajectory = load_csv(robot_path / "trajectory.csv")

        if whole_position.shape[1] < 2:
            raise ValueError(f"Expected at least 2 columns in {robot_path / 'whole_position_history.csv'}")
        if goal_position.shape[1] < 3:
            raise ValueError(f"Expected at least 3 columns in {robot_path / 'goal_position.csv'}")
        if trajectory.shape[1] < 3:
            raise ValueError(f"Expected at least 3 columns in {robot_path / 'trajectory.csv'}")

        positions.append(np.round(whole_position[:, :2], 1))
        goal_positions.append(np.round(goal_position[:, 1:3], 1))
        trajectories.append(trajectory[:, :3])

    cluster_data = load_csv(reference_robot / "posterior_pdf_3D_cluster.csv")
    if cluster_data.shape[1] < 3:
        raise ValueError(f"Expected cluster data in {reference_robot / 'posterior_pdf_3D_cluster.csv'}")
    belief_maps = cluster_data[:, 2:]

    intensity_maps = load_csv(reference_robot / "posterior_pdf_3D.csv")
    clusters_geometric = load_csv(reference_robot / "cluster_sizes_geometric.csv")
    clusters_informative = load_csv(reference_robot / "cluster_sizes_informative.csv")
    source_xy = parse_source_position(reference_robot / "source_pos.csv", params)

    return ExperimentData(
        base_path=base_path,
        exp_folder=exp_folder,
        robot_ids=robot_ids,
        map_nbr=map_nbr,
        obstacles=obstacles,
        positions=positions,
        goal_positions=goal_positions,
        belief_maps=belief_maps,
        intensity_maps=intensity_maps,
        clusters_geometric=clusters_geometric,
        clusters_informative=clusters_informative,
        source_xy=source_xy,
        trajectories=trajectories,
    )


def draw_walls(ax: plt.Axes) -> None:
    wall_specs = [
        (-WALL_THICKNESS, -WALL_THICKNESS, 16.0 + WALL_THICKNESS, WALL_THICKNESS),
        (0.0, 4.0, 16.0 + WALL_THICKNESS, WALL_THICKNESS),
        (-WALL_THICKNESS, 0.0, WALL_THICKNESS, 4.0 + WALL_THICKNESS),
        (16.0, -WALL_THICKNESS, WALL_THICKNESS, 4.0 + WALL_THICKNESS),
    ]
    for x, y, width, height in wall_specs:
        ax.add_patch(
            mpatches.Rectangle(
                (x, y),
                width,
                height,
                facecolor=DARK_GRAY,
                edgecolor="#15191f",
                linewidth=0.5,
                zorder=4,
            )
        )


def draw_obstacles(ax: plt.Axes, obstacles: list[tuple[float, float, float, float]]) -> None:
    for x1, y1, x2, y2 in obstacles:
        ax.add_patch(
            mpatches.Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                facecolor=DARK_GRAY,
                edgecolor="#15191f",
                linewidth=0.6,
                zorder=5,
            )
        )


def create_cell_patches(
    ax: plt.Axes,
    rows: int,
    cols: int,
    show_cell_indices: bool,
) -> list[list[mpatches.Rectangle]]:
    x_edges = np.linspace(CELL_X_MIN, CELL_X_MAX, cols + 1)
    y_edges = np.linspace(CELL_Y_MIN, CELL_Y_MAX, rows + 1)

    cell_patches: list[list[mpatches.Rectangle]] = []
    for row in range(rows):
        patch_row: list[mpatches.Rectangle] = []
        for col in range(cols):
            x = x_edges[col]
            y = y_edges[row]
            width = x_edges[col + 1] - x
            height = y_edges[row + 1] - y
            patch = mpatches.Rectangle(
                (x, y),
                width,
                height,
                facecolor=WHITE,
                edgecolor="white",
                linewidth=0.25,
                zorder=1,
            )
            ax.add_patch(patch)
            patch_row.append(patch)

            if show_cell_indices:
                index = row * cols + col + 1
                ax.text(
                    x + width / 2.0,
                    y + height / 2.0,
                    str(index),
                    fontsize=8,
                    ha="center",
                    va="center",
                    color=(0.3, 0.3, 0.3),
                    zorder=2,
                )
        cell_patches.append(patch_row)
    return cell_patches


def update_cells(
    cell_patches: list[list[mpatches.Rectangle]],
    belief_map: np.ndarray,
    intensity_map: np.ndarray,
) -> None:
    rows = len(cell_patches)
    cols = len(cell_patches[0]) if rows else 0
    expected_size = rows * cols
    if belief_map.size < expected_size or intensity_map.size < expected_size:
        raise ValueError(
            f"Expected {expected_size} map cells, got {belief_map.size} belief and {intensity_map.size} intensity"
        )

    min_int = np.nanmin(intensity_map)
    max_int = np.nanmax(intensity_map)
    if max_int > min_int:
        normalized_intensity = (intensity_map - min_int) / (max_int - min_int)
    else:
        normalized_intensity = np.zeros_like(intensity_map)

    min_visible_intensity = 0.2
    for row in range(rows):
        for col in range(cols):
            map_idx = row * cols + col
            class_value = int(round(float(belief_map[map_idx])))
            base_color = CLASS_COLORS.get(class_value, WHITE)
            intensity = float(np.clip(normalized_intensity[map_idx], 0.0, 1.0))
            adjusted_intensity = min_visible_intensity + (1.0 - min_visible_intensity) * intensity
            blended_color = base_color * adjusted_intensity + WHITE * (1.0 - adjusted_intensity)
            cell_patches[row][col].set_facecolor(blended_color)


def set_axis_style(ax: plt.Axes) -> None:
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(*WORLD_X_LIMITS)
    ax.set_ylim(*WORLD_Y_LIMITS)
    ax.set_facecolor(AX_FACE)
    ax.set_xlabel("X (m)", color=TEXT_COLOR, labelpad=8)
    ax.set_ylabel("Y (m)", color=TEXT_COLOR, labelpad=8)
    ax.set_xticks(np.arange(0, 17, 2))
    ax.set_yticks(np.arange(0, 5, 1))
    ax.grid(True, color=GRID_COLOR, linewidth=0.6, alpha=0.65, zorder=0)
    ax.tick_params(colors="#4c5663", labelsize=9)
    for spine in ax.spines.values():
        spine.set_color(SPINE_COLOR)
        spine.set_linewidth(0.9)


def set_line_data(line, x_values: np.ndarray, y_values: np.ndarray) -> None:
    line.set_data(x_values, y_values)


def print_cluster_counts(belief_map: np.ndarray) -> None:
    print(f"White cells (value=0): {np.sum(belief_map == 0)}")
    print(f"Red cells   (value=1): {np.sum(belief_map == 1)}")
    print(f"Blue cells  (value=2): {np.sum(belief_map == 2)}")
    print(f"Green cells (value=3): {np.sum(belief_map == 3)}")
    print("=================================================")


def make_writer(
    output_path: Path,
    fps: int,
    codec: str,
    bitrate: int,
) -> animation.AbstractMovieWriter | None:
    if not animation.FFMpegWriter.isAvailable():
        print("Warning: ffmpeg is not available; video output disabled.")
        return None
    return animation.FFMpegWriter(
        fps=fps,
        codec=codec,
        bitrate=bitrate,
        extra_args=["-pix_fmt", "yuv420p"],
    )


def animate(data: ExperimentData, args: argparse.Namespace) -> None:
    rows = args.rows
    cols = args.cols
    repr_vel = args.repr_vel
    video_fps = args.fps

    fig, ax = plt.subplots(figsize=(13.5, 5.2), facecolor="white", constrained_layout=True)
    set_axis_style(ax)
    ax.set_title(
        f"FSM cycle: 1   |   map {data.map_nbr}",
        fontsize=16,
        fontweight="bold",
        color=TEXT_COLOR,
        pad=12,
    )

    cell_patches = create_cell_patches(ax, rows, cols, args.show_cell_indices)
    draw_walls(ax)
    draw_obstacles(ax, data.obstacles)

    source_x, source_y = data.source_xy
    ax.plot(
        source_x,
        source_y,
        marker="*",
        markersize=22,
        markeredgecolor="white",
        markerfacecolor="white",
        linestyle="None",
        zorder=7,
    )
    ax.plot(
        source_x,
        source_y,
        marker="*",
        markersize=17,
        markeredgecolor="black",
        markerfacecolor="black",
        linestyle="None",
        zorder=8,
    )

    traj_colors = ["#d95f02", "#1f77b4", "#2ca25f"]
    robot_colors = traj_colors

    traj_lines = []
    traj_dots = []
    for idx, _robot_id in enumerate(data.robot_ids):
        color = traj_colors[idx % len(traj_colors)]
        (line,) = ax.plot([], [], "-", color=color, linewidth=2.4, alpha=0.9, zorder=6)
        (dots,) = ax.plot([], [], ".", color=color, markersize=7, alpha=0.75, zorder=7)
        traj_lines.append(line)
        traj_dots.append(dots)

    robot_markers = []
    goal_markers = []
    for idx, _robot_id in enumerate(data.robot_ids):
        (robot_marker,) = ax.plot(
            [],
            [],
            "o",
            markersize=9.5,
            markerfacecolor=robot_colors[idx % len(robot_colors)],
            markeredgecolor="black",
            markeredgewidth=1.2,
            linestyle="None",
            zorder=9,
        )
        (goal_marker,) = ax.plot(
            [],
            [],
            "x",
            markersize=11,
            markeredgewidth=2.2,
            color="#111827",
            linestyle="None",
            zorder=9,
        )
        robot_markers.append(robot_marker)
        goal_markers.append(goal_marker)

    legend_handles = [
        Line2D(
            [0],
            [0],
            color=traj_colors[idx % len(traj_colors)],
            lw=2.4,
            marker="o",
            markerfacecolor=robot_colors[idx % len(robot_colors)],
            markeredgecolor="black",
            markersize=7,
            label=f"Robot {robot_id}",
        )
        for idx, robot_id in enumerate(data.robot_ids)
    ]
    legend_handles.extend(
        [
            Line2D([0], [0], marker="x", color="#111827", lw=0, markersize=9, markeredgewidth=2, label="Goal"),
            Line2D([0], [0], marker="*", color="black", lw=0, markersize=12, label="Source"),
        ]
    )
    ax.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.18),
        ncol=len(legend_handles),
        frameon=True,
        facecolor="white",
        edgecolor="#cfd7e3",
        fontsize=9,
        handlelength=2.0,
        columnspacing=1.2,
    )

    state = {"paused": False, "stop": False}

    def on_key(event) -> None:
        if event.key == " ":
            state["paused"] = not state["paused"]
            print("Animation paused. Press space again to resume." if state["paused"] else "Animation resumed.")
        elif event.key == "q":
            state["stop"] = True

    if args.show:
        fig.canvas.mpl_connect("key_press_event", on_key)
        plt.show(block=False)

    output_path = args.output
    writer = None if args.no_video else make_writer(output_path, video_fps, args.codec, args.bitrate)
    writer_context = (
        writer.saving(fig, str(output_path), dpi=args.dpi) if writer is not None else nullcontext()
    )

    goal_idx = np.ones(len(data.robot_ids), dtype=int) - 1
    reached_goals = np.zeros(len(data.robot_ids), dtype=bool)
    t_idx = np.ones(len(data.robot_ids), dtype=int) - 1
    done = np.zeros(len(data.robot_ids), dtype=bool)
    last_pos = np.zeros((len(data.robot_ids), 2))
    for idx, position_history in enumerate(data.positions):
        if len(position_history) > 0:
            last_pos[idx, :] = position_history[0, :]

    n_cluster_updates = len(data.belief_maps)
    h_vals_geometric = np.zeros(n_cluster_updates)
    h_vals_informative = np.zeros(n_cluster_updates)
    goal_update_count = 0
    score_text_geometric = None
    score_text_informative = None

    max_frames = args.max_frames if args.max_frames is not None else 10000
    completed = False

    with writer_context:
        for _frame_t in range(max_frames):
            if state["stop"]:
                break

            if np.all(reached_goals):
                goal_idx += 1
                reached_goals[:] = False
                goal_update_count += 1

                if goal_update_count <= n_cluster_updates:
                    map_idx = goal_update_count - 1
                    belief_map = data.belief_maps[map_idx, :]
                    intensity_map = data.intensity_maps[map_idx, :]
                    update_cells(cell_patches, belief_map, intensity_map)

                    if map_idx < len(data.clusters_geometric):
                        h_vals_geometric[map_idx] = normalized_entropy(data.clusters_geometric[map_idx, :])
                    if map_idx < len(data.clusters_informative):
                        h_vals_informative[map_idx] = normalized_entropy(data.clusters_informative[map_idx, :])

                    if score_text_geometric is not None:
                        score_text_geometric.remove()
                    if score_text_informative is not None:
                        score_text_informative.remove()

                    score_text_geometric = ax.text(
                        0.5,
                        -0.7,
                        f"Score [GEOMETRIC]:    {h_vals_geometric[map_idx] * 100:.2f}%",
                        fontsize=12,
                        fontweight="bold",
                        color=TEXT_COLOR,
                        clip_on=False,
                        bbox=dict(
                            boxstyle="round,pad=0.35,rounding_size=0.12",
                            facecolor="white",
                            edgecolor="#cfd7e3",
                            linewidth=0.8,
                            alpha=0.95,
                        ),
                    )
                    score_text_informative = ax.text(
                        0.5,
                        -1.2,
                        f"Score [INFORMATIVE]: {h_vals_informative[map_idx] * 100:.2f}%",
                        fontsize=12,
                        fontweight="bold",
                        color=TEXT_COLOR,
                        clip_on=False,
                        bbox=dict(
                            boxstyle="round,pad=0.35,rounding_size=0.12",
                            facecolor="white",
                            edgecolor="#cfd7e3",
                            linewidth=0.8,
                            alpha=0.95,
                        ),
                    )

                    print_cluster_counts(belief_map)
                    ax.set_title(
                        f"FSM cycle: {goal_update_count}   |   map {data.map_nbr}",
                        fontsize=16,
                        fontweight="bold",
                        color=TEXT_COLOR,
                        pad=12,
                    )

                    for robot_idx, trajectory in enumerate(data.trajectories):
                        cycles = trajectory[:, 0]
                        mask = np.abs(cycles - goal_update_count) < 1e-3
                        curr_x = trajectory[mask, 1]
                        curr_y = trajectory[mask, 2]
                        set_line_data(traj_lines[robot_idx], curr_x, curr_y)
                        set_line_data(traj_dots[robot_idx], curr_x, curr_y)

            for robot_idx, _robot_id in enumerate(data.robot_ids):
                if done[robot_idx] or t_idx[robot_idx] >= len(data.positions[robot_idx]):
                    done[robot_idx] = True
                    continue

                curr_pos = data.positions[robot_idx][t_idx[robot_idx], :]

                if goal_idx[robot_idx] < len(data.goal_positions[robot_idx]):
                    goal = data.goal_positions[robot_idx][goal_idx[robot_idx], :]
                    distance = np.linalg.norm(curr_pos - goal)
                    reached_goals[robot_idx] = distance < 0.15
                    goal_markers[robot_idx].set_data([goal[0]], [goal[1]])

                if not reached_goals[robot_idx]:
                    robot_markers[robot_idx].set_data([curr_pos[0]], [curr_pos[1]])
                    last_pos[robot_idx, :] = curr_pos
                    t_idx[robot_idx] += repr_vel
                    if t_idx[robot_idx] >= len(data.positions[robot_idx]):
                        t_idx[robot_idx] = len(data.positions[robot_idx]) - 1
                        done[robot_idx] = True
                else:
                    robot_markers[robot_idx].set_data(
                        [last_pos[robot_idx, 0]],
                        [last_pos[robot_idx, 1]],
                    )

            fig.canvas.draw()
            if args.show:
                plt.pause(args.pause)
                while state["paused"] and not state["stop"]:
                    plt.pause(0.1)

            if writer is not None:
                writer.grab_frame()

            if np.all(done):
                completed = True
                break

        if completed:
            draw_final_result(
                ax,
                data,
                h_vals_geometric,
                h_vals_informative,
                score_text_geometric,
                score_text_informative,
            )
            fig.canvas.draw()
            for _ in range(args.final_hold_frames):
                if writer is not None:
                    writer.grab_frame()

    if writer is not None:
        print(f"Video saved to {output_path}")
    elif not args.no_video:
        print("Video was requested but not written.")

    if args.show and not state["stop"]:
        print("Animation complete. Close the figure window to exit.")
        plt.show(block=True)
    else:
        plt.close(fig)


def draw_final_result(
    ax: plt.Axes,
    data: ExperimentData,
    h_vals_geometric: np.ndarray,
    h_vals_informative: np.ndarray,
    score_text_geometric,
    score_text_informative,
) -> None:
    expected_path = data.exp_folder / str(data.robot_ids[-1]) / "expected_a_posteriori.csv"
    expected_data = load_csv(expected_path)
    if expected_data.shape[1] < 3 or expected_data.shape[0] == 0:
        print(f"Warning: could not read estimated source from {expected_path}")
        return

    estimated_x = expected_data[-1, 1]
    estimated_y = expected_data[-1, 2]
    source_x, source_y = data.source_xy

    ax.plot(
        estimated_x,
        estimated_y,
        marker="*",
        markersize=19,
        markeredgecolor="black",
        markerfacecolor=YELLOW,
        linestyle="None",
        zorder=10,
    )

    x_error = abs(estimated_x - source_x)
    y_error = abs(estimated_y - source_y)
    if x_error < 1.0 and y_error < 0.5:
        msg = "SOURCE FOUND"
        color = (0.1, 0.6, 0.1)
    else:
        msg = "SOURCE NOT FOUND"
        color = (0.8, 0.1, 0.1)

    ax.text(
        9.5,
        -1.0,
        msg,
        fontsize=13,
        fontweight="bold",
        color=color,
        clip_on=False,
        bbox=dict(
            boxstyle="round,pad=0.4,rounding_size=0.12",
            facecolor="white",
            edgecolor="#cfd7e3",
            linewidth=0.8,
            alpha=0.96,
        ),
    )

    if score_text_geometric is not None:
        score_text_geometric.remove()
    if score_text_informative is not None:
        score_text_informative.remove()

    ax.text(
        0.5,
        -0.7,
        f"Score [GEOMETRIC] (avg):    {np.mean(h_vals_geometric) * 100:.2f}%",
        fontsize=12,
        fontweight="bold",
        color=TEXT_COLOR,
        clip_on=False,
        bbox=dict(
            boxstyle="round,pad=0.35,rounding_size=0.12",
            facecolor="white",
            edgecolor="#cfd7e3",
            linewidth=0.8,
            alpha=0.95,
        ),
    )
    ax.text(
        0.5,
        -1.2,
        f"Score [INFORMATIVE] (avg): {np.mean(h_vals_informative) * 100:.2f}%",
        fontsize=12,
        fontweight="bold",
        color=TEXT_COLOR,
        clip_on=False,
        bbox=dict(
            boxstyle="round,pad=0.35,rounding_size=0.12",
            facecolor="white",
            edgecolor="#cfd7e3",
            linewidth=0.8,
            alpha=0.95,
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Animate multi-robot cluster maps and trajectories.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=DEFAULT_BASE_PATH,
        help="Base folder, experiment folder, or robot folder.",
    )
    parser.add_argument(
        "--experiment-index",
        type=int,
        default=0,
        help="Subfolder index to use when path is a base folder.",
    )
    parser.add_argument(
        "--robot-ids",
        nargs="+",
        type=int,
        default=list(DEFAULT_ROBOT_IDS),
        help="Robot IDs to animate.",
    )
    parser.add_argument("--repr-vel", type=int, default=DEFAULT_REPR_VEL, help="Position steps skipped per frame.")
    parser.add_argument("--rows", type=int, default=20, help="Number of map rows.")
    parser.add_argument("--cols", type=int, default=20, help="Number of map columns.")
    parser.add_argument("--show-cell-indices", action="store_true", help="Draw linear cell indices.")
    parser.add_argument("--no-video", action="store_true", help="Disable video output.")
    parser.add_argument("--no-show", dest="show", action="store_false", help="Do not show a GUI window.")
    parser.set_defaults(show=True)
    parser.add_argument("--pause", type=float, default=1e-5, help="GUI pause between frames.")
    parser.add_argument("--max-frames", type=int, default=None, help="Maximum animation frames.")
    parser.add_argument("--final-hold-frames", type=int, default=45, help="Extra final frames in the video.")
    parser.add_argument("--fps", type=int, default=DEFAULT_VIDEO_FPS, help="Output video frames per second.")
    parser.add_argument("--dpi", type=int, default=DEFAULT_VIDEO_DPI, help="Video DPI.")
    parser.add_argument("--codec", default="mpeg4", help="FFmpeg codec.")
    parser.add_argument("--bitrate", type=int, default=3200, help="Output video bitrate in kbps.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output video path. Defaults to <base_path>/animation.avi.",
    )
    args = parser.parse_args()
    args.robot_ids = tuple(args.robot_ids)
    return args


def main() -> None:
    args = parse_args()
    data = load_experiment(args.path, args.robot_ids, args.experiment_index)
    if args.output is None:
        args.output = data.base_path / "animation.avi"
    else:
        args.output = args.output.expanduser().resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    print(f"Experiment folder: {data.exp_folder}")
    print(f"map_nbr from param_log: {data.map_nbr}")
    print(f"Loaded obstacles: {len(data.obstacles)}")
    animate(data, args)


if __name__ == "__main__":
    main()
