#!/usr/bin/env python3
"""
Interactive multi-robot experiment viewer.

Shows one experiment iteration at a time:
  1. robot trajectories from position_history.csv, colored by gas measurements
     from measurement_history.csv column 2
  2. one individual posterior_pdf.csv belief map per robot and one merged_pdf.csv
     belief map
  3. region allocation from region_allocation.csv
  4. team wind field from wind_field_u.csv and wind_field_v.csv
  5. wind uncertainty derived from wind_field_w.csv

Controls:
  right / space / n / enter : next iteration
  left / backspace / p      : previous iteration
  home                      : first iteration
  end                       : last iteration
  q / escape                : close
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import BoundaryNorm, LinearSegmentedColormap, ListedColormap, Normalize
from matplotlib.lines import Line2D


DEFAULT_ROOT = Path(
    "/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/"
    "Results_2026-06-30/11-50"
)

ROBOT_COLORS = [
    "#1a6faf",
    "#d95f02",
    "#1b7837",
    "#7b3294",
    "#b2182b",
    "#00897b",
]
DARK_GRAY = "#3a3a3a"
EPS = 1e-12

TITLE_FONT_SIZE = 14
SUPTITLE_FONT_SIZE = 18
AXIS_LABEL_SIZE = 13
TICK_LABEL_SIZE = 11
LEGEND_FONT_SIZE = 10
COLORBAR_LABEL_SIZE = 12
COLORBAR_TICK_SIZE = 11
SOURCE_MARKER_SIZE = 15
TRAJECTORY_SCATTER_SIZE = 38
TRAJECTORY_LINE_WIDTH = 2.8
TRAJECTORY_GUIDE_LINE_WIDTH = 1.7
START_MARKER_SIZE = 80
CURRENT_MARKER_SIZE = 90
GOAL_MARKER_SIZE = 120
LEGEND_MARKER_SIZE = 8


@dataclass(frozen=True)
class GridInfo:
    nx: int
    ny: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def x_edges(self) -> np.ndarray:
        return np.linspace(self.x_min, self.x_max, self.nx + 1)

    @property
    def y_edges(self) -> np.ndarray:
        return np.linspace(self.y_min, self.y_max, self.ny + 1)


@dataclass
class RobotLog:
    name: str
    folder: Path
    positions: np.ndarray
    gas: np.ndarray
    goal_positions: np.ndarray | None
    posterior: np.ndarray
    merged: np.ndarray | None
    region_allocation: np.ndarray | None
    wind_u: np.ndarray | None
    wind_v: np.ndarray | None
    wind_w: np.ndarray | None


def load_csv(path: Path, *, required: bool = True) -> np.ndarray | None:
    if not path.exists():
        if required:
            raise FileNotFoundError(f"Cannot find required file: {path}")
        return None

    data = np.genfromtxt(path, delimiter=",")
    if data.size == 0:
        if required:
            raise ValueError(f"CSV file is empty: {path}")
        return None
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data


def parse_numbers(text: str) -> list[float]:
    return [float(v) for v in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", text)]


def load_source_position(root: Path) -> tuple[float, float] | None:
    for path in (root / "source_position.txt", root / "source_pos.csv"):
        if not path.exists():
            continue
        values = parse_numbers(path.read_text(encoding="utf-8"))
        if len(values) >= 2:
            return values[0], values[1]
    return None


def parse_param_log(path: Path | None, pdf_cols: int) -> GridInfo:
    fallback = GridInfo(nx=20, ny=20, x_min=3.0, x_max=12.75, y_min=0.0, y_max=4.0)
    if path is None or not path.exists():
        return infer_grid_from_columns(pdf_cols, fallback)

    nbins: list[int] | None = None
    min_maxs: list[float] | None = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        tokens = raw_line.split()
        if not tokens:
            continue
        if tokens[0] == "nbins":
            nbins = [int(float(v)) for v in tokens[1:]]
        elif tokens[0] == "min_maxs":
            min_maxs = [float(v) for v in tokens[1:]]

    if nbins and min_maxs and len(nbins) >= 3 and len(min_maxs) >= 6:
        # param_log.csv stores Q first, then source x and source y.
        candidate = GridInfo(
            nx=nbins[1],
            ny=nbins[2],
            x_min=min_maxs[2],
            x_max=min_maxs[3],
            y_min=min_maxs[4],
            y_max=min_maxs[5],
        )
        if candidate.nx * candidate.ny == pdf_cols:
            return candidate

    if nbins and len(nbins) >= 2 and nbins[0] * nbins[1] == pdf_cols:
        return GridInfo(
            nx=nbins[0],
            ny=nbins[1],
            x_min=fallback.x_min,
            x_max=fallback.x_max,
            y_min=fallback.y_min,
            y_max=fallback.y_max,
        )

    return infer_grid_from_columns(pdf_cols, fallback)


def infer_grid_from_columns(cols: int, fallback: GridInfo) -> GridInfo:
    if cols == fallback.nx * fallback.ny:
        return fallback

    root = int(round(math.sqrt(cols)))
    if root * root == cols:
        return GridInfo(root, root, fallback.x_min, fallback.x_max, fallback.y_min, fallback.y_max)

    if cols % fallback.ny == 0:
        return GridInfo(cols // fallback.ny, fallback.ny, fallback.x_min, fallback.x_max, fallback.y_min, fallback.y_max)

    raise ValueError(f"Cannot infer a 2D grid shape from {cols} columns")


def infer_wind_shape(flat_cols: int, belief_grid: GridInfo) -> tuple[int, int]:
    if flat_cols == belief_grid.nx * belief_grid.ny:
        return belief_grid.ny, belief_grid.nx
    if flat_cols % belief_grid.ny == 0:
        return belief_grid.ny, flat_cols // belief_grid.ny

    root = int(round(math.sqrt(flat_cols)))
    if root * root == flat_cols:
        return root, root

    factors = [(d, flat_cols // d) for d in range(1, int(math.sqrt(flat_cols)) + 1) if flat_cols % d == 0]
    if not factors:
        raise ValueError(f"Cannot infer wind grid shape from {flat_cols} columns")
    return min(factors, key=lambda pair: abs(pair[0] - pair[1]))


def robot_sort_key(path: Path) -> tuple[int, str]:
    numbers = re.findall(r"\d+", path.name)
    return (int(numbers[-1]) if numbers else 0, path.name)


def discover_robot_dirs(k_dir: Path) -> list[Path]:
    robot_dirs = [
        path
        for path in k_dir.iterdir()
        if path.is_dir()
        and (path / "position_history.csv").exists()
        and (path / "measurement_history.csv").exists()
        and (path / "posterior_pdf.csv").exists()
    ]
    return sorted(robot_dirs, key=robot_sort_key)


def load_robot_log(folder: Path) -> RobotLog:
    positions_raw = load_csv(folder / "position_history.csv")
    measurements = load_csv(folder / "measurement_history.csv")
    goal_positions_raw = load_csv(folder / "goal_position.csv", required=False)
    posterior = load_csv(folder / "posterior_pdf.csv")
    merged = load_csv(folder / "merged_pdf.csv", required=False)
    region_allocation = load_csv(folder / "region_allocation.csv", required=False)
    wind_u = load_csv(folder / "wind_field_u.csv", required=False)
    wind_v = load_csv(folder / "wind_field_v.csv", required=False)
    wind_w = load_csv(folder / "wind_field_w.csv", required=False)

    assert positions_raw is not None
    assert measurements is not None
    assert posterior is not None

    if positions_raw.shape[1] < 3:
        raise ValueError(f"Expected x,y in {folder / 'position_history.csv'}")
    if measurements.shape[1] < 2:
        raise ValueError(f"Expected gas measurement in column 2 of {folder / 'measurement_history.csv'}")

    n_history = min(len(positions_raw), len(measurements))
    positions = positions_raw[:n_history, 1:3]
    gas = measurements[:n_history, 1]

    goal_positions = None
    if goal_positions_raw is not None:
        if goal_positions_raw.shape[1] >= 3:
            goal_positions = goal_positions_raw[:, 1:3]
        elif goal_positions_raw.shape[1] == 2:
            goal_positions = goal_positions_raw[:, :2]
        else:
            raise ValueError(f"Expected x,y in {folder / 'goal_position.csv'}")

    return RobotLog(
        name=folder.name.replace("log_STE_multi_", "Robot "),
        folder=folder,
        positions=positions,
        gas=gas,
        goal_positions=goal_positions,
        posterior=posterior,
        merged=merged,
        region_allocation=region_allocation,
        wind_u=wind_u,
        wind_v=wind_v,
        wind_w=wind_w,
    )


def load_obstacles(path: Path | None) -> list[tuple[float, float, float, float]]:
    if path is None or not path.exists():
        return []

    rows: list[tuple[float, float, float, float]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        values = parse_numbers(line)
        if len(values) >= 4:
            x0, x1, y0, y1 = values[:4]
            rows.append((min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0)))
    return rows


def row_at(data: np.ndarray | None, iteration: int) -> np.ndarray | None:
    if data is None or data.size == 0:
        return None
    return data[min(iteration, data.shape[0] - 1)]


def normalize_map(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(values)
    if not np.any(valid):
        return np.zeros_like(values)

    mn = float(np.nanmin(values[valid]))
    mx = float(np.nanmax(values[valid]))
    if mx <= mn:
        return np.zeros_like(values)
    return (values - mn) / (mx - mn)


def reshape_map(values: np.ndarray, grid: GridInfo) -> np.ndarray:
    if values.size != grid.nx * grid.ny:
        raise ValueError(f"Expected {grid.nx * grid.ny} map values, got {values.size}")
    return values.reshape(grid.ny, grid.nx)


def reshape_wind(values: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    ny, nx = shape
    if values.size != ny * nx:
        raise ValueError(f"Expected {ny * nx} wind values, got {values.size}")
    return values.reshape(ny, nx)


def make_line_collection(points: np.ndarray, values: np.ndarray, norm: Normalize, cmap: str) -> LineCollection:
    if len(points) < 2:
        return LineCollection([], cmap=cmap, norm=norm)

    segments = np.stack([points[:-1], points[1:]], axis=1)
    collection = LineCollection(segments, cmap=cmap, norm=norm, linewidths=TRAJECTORY_LINE_WIDTH, alpha=0.75)
    collection.set_array(values[:-1])
    return collection


def style_map_ax(
    ax: plt.Axes,
    grid: GridInfo,
    source_xy: tuple[float, float] | None,
    obstacles: list[tuple[float, float, float, float]],
) -> None:
    for x, y, width, height in obstacles:
        ax.add_patch(
            mpatches.Rectangle((x, y), width, height, facecolor=DARK_GRAY, edgecolor="none", zorder=5)
        )

    if source_xy is not None:
        ax.plot(
            source_xy[0],
            source_xy[1],
            "*",
            markersize=SOURCE_MARKER_SIZE,
            markerfacecolor="black",
            markeredgecolor="white",
            markeredgewidth=0.8,
            zorder=8,
        )

    ax.set_xlim(grid.x_min, grid.x_max)
    ax.set_ylim(grid.y_min, grid.y_max)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)", fontsize=AXIS_LABEL_SIZE)
    ax.set_ylabel("y (m)", fontsize=AXIS_LABEL_SIZE)
    ax.tick_params(axis="both", labelsize=TICK_LABEL_SIZE)
    ax.grid(True, alpha=0.18, linewidth=0.5)


class ExperimentViewer:
    def __init__(
        self,
        robots: list[RobotLog],
        grid: GridInfo,
        wind_shape: tuple[int, int] | None,
        source_xy: tuple[float, float] | None,
        obstacles: list[tuple[float, float, float, float]],
        *,
        history_mode: str,
        uncertainty_mode: str,
        fig_scale: float,
        save_dir: Path | None = None,
    ) -> None:
        self.robots = robots
        self.grid = grid
        self.wind_shape = wind_shape
        self.source_xy = source_xy
        self.obstacles = obstacles
        self.history_mode = history_mode
        self.uncertainty_mode = uncertainty_mode
        self.fig_scale = fig_scale
        self.save_dir = save_dir
        self.iteration = 0

        self.robot_cmaps = [
            LinearSegmentedColormap.from_list("", ["#ffffff", ROBOT_COLORS[i % len(ROBOT_COLORS)]], N=256)
            for i in range(len(robots))
        ]
        self.merged_cmap = LinearSegmentedColormap.from_list("", ["#ffffff", "#1e2d40"], N=256)
        region_colors = ["#f0f0f0"] + [ROBOT_COLORS[i % len(ROBOT_COLORS)] for i in range(len(robots))]
        self.region_cmap = ListedColormap(region_colors)
        self.region_norm = BoundaryNorm(
            np.arange(-0.5, len(region_colors) + 0.5, 1.0),
            self.region_cmap.N,
            clip=True,
        )

        self.n_iterations = self._count_iterations()
        self.gas_norm = self._gas_norm()

        n_robot_cols = max(len(robots), 1)
        n_panel_cols = max(n_robot_cols, 4)
        n_grid_cols = 4 * n_robot_cols // math.gcd(4, n_robot_cols)
        self.fig = plt.figure(figsize=(6.2 * n_panel_cols * fig_scale, 15.0 * fig_scale), constrained_layout=True)
        gs = self.fig.add_gridspec(3, n_grid_cols)

        robot_edges = np.linspace(0, n_grid_cols, len(robots) + 1, dtype=int)
        self.ax_trajectories = [
            self.fig.add_subplot(gs[0, robot_edges[idx] : robot_edges[idx + 1]])
            for idx in range(len(robots))
        ]
        self.ax_beliefs = [
            self.fig.add_subplot(gs[1, robot_edges[idx] : robot_edges[idx + 1]])
            for idx in range(len(robots))
        ]

        panel_edges = np.linspace(0, n_grid_cols, 5, dtype=int)
        self.ax_merged = self.fig.add_subplot(gs[2, panel_edges[0] : panel_edges[1]])
        self.ax_region = self.fig.add_subplot(gs[2, panel_edges[1] : panel_edges[2]])
        self.ax_wind = self.fig.add_subplot(gs[2, panel_edges[2] : panel_edges[3]])
        self.ax_uncertainty = self.fig.add_subplot(gs[2, panel_edges[3] : panel_edges[4]])

        self.colorbars = []
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def _count_iterations(self) -> int:
        counts = [robot.posterior.shape[0] for robot in self.robots]
        merged_counts = [robot.merged.shape[0] for robot in self.robots if robot.merged is not None]
        if merged_counts:
            counts.append(max(merged_counts))
        if not counts:
            raise ValueError("No iteration-indexed maps were loaded")
        return max(1, min(counts))

    def _gas_norm(self) -> Normalize:
        all_gas = [robot.gas[np.isfinite(robot.gas)] for robot in self.robots if robot.gas.size]
        if not all_gas:
            return Normalize(vmin=0.0, vmax=1.0)
        values = np.concatenate(all_gas)
        vmin = float(np.min(values))
        vmax = float(np.max(values))
        if vmax <= vmin:
            vmax = vmin + 1.0
        return Normalize(vmin=vmin, vmax=vmax)

    def history_end_index(self, robot: RobotLog) -> int:
        if self.history_mode == "full":
            return len(robot.positions)

        goal_xy = self.robot_goal_xy(robot)
        if goal_xy is not None and len(robot.positions) > 0:
            target = np.asarray(goal_xy, dtype=float)
            distances = np.linalg.norm(robot.positions - target, axis=1)
            valid_indices = np.flatnonzero(np.isfinite(distances))
            if valid_indices.size > 0:
                nearest = valid_indices[np.argmin(distances[valid_indices])]
                return int(nearest + 1)

        return max(1, math.ceil(self.iteration * len(robot.positions) / self.n_iterations))

    def merged_row(self) -> np.ndarray | None:
        candidates = [row_at(robot.merged, self.iteration) for robot in self.robots if robot.merged is not None]
        candidates = [row for row in candidates if row is not None]
        if not candidates:
            return None
        return np.mean(np.vstack(candidates), axis=0)

    def region_allocation_row(self) -> np.ndarray | None:
        expected_cells = self.grid.nx * self.grid.ny
        for robot in self.robots:
            row = row_at(robot.region_allocation, self.iteration)
            if row is None:
                continue
            if row.size == expected_cells + 2:
                return row[2:]
            if row.size == expected_cells:
                return row
        return None

    def robot_goal_xy(self, robot: RobotLog) -> tuple[float, float] | None:
        goal = row_at(robot.goal_positions, self.iteration)
        if goal is not None and goal.size >= 2 and np.all(np.isfinite(goal[:2])):
            return float(goal[0]), float(goal[1])

        expected_cells = self.grid.nx * self.grid.ny
        allocation = row_at(robot.region_allocation, self.iteration)
        if (
            allocation is not None
            and allocation.size == expected_cells + 2
            and np.all(np.isfinite(allocation[:2]))
        ):
            return float(allocation[0]), float(allocation[1])

        return None

    def wind_rows(self) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        if self.wind_shape is None:
            return None

        u_rows: list[np.ndarray] = []
        v_rows: list[np.ndarray] = []
        w_rows: list[np.ndarray] = []
        for robot in self.robots:
            u = row_at(robot.wind_u, self.iteration)
            v = row_at(robot.wind_v, self.iteration)
            w = row_at(robot.wind_w, self.iteration)
            if u is not None and v is not None:
                u_rows.append(u)
                v_rows.append(v)
                if w is not None:
                    w_rows.append(w)

        if not u_rows or not v_rows:
            return None

        u_mean = np.mean(np.vstack(u_rows), axis=0)
        v_mean = np.mean(np.vstack(v_rows), axis=0)
        if w_rows:
            w_mean = np.mean(np.vstack(w_rows), axis=0)
        else:
            w_mean = np.zeros_like(u_mean)
        return u_mean, v_mean, w_mean

    def draw(self) -> None:
        self.clear_colorbars()
        self.draw_trajectories()
        self.draw_individual_beliefs()
        self.draw_merged_belief()
        self.draw_region_allocation()
        self.draw_wind()

        self.fig.suptitle(
            f"Multi-robot experiment - iteration {self.iteration + 1}/{self.n_iterations}",
            fontsize=SUPTITLE_FONT_SIZE,
            fontweight="bold",
        )
        self.fig.canvas.draw_idle()

    def clear_colorbars(self) -> None:
        for colorbar in self.colorbars:
            try:
                colorbar.remove()
            except (AttributeError, KeyError, ValueError):
                pass
        self.colorbars = []

    def add_colorbar(self, mappable, ax, **kwargs):
        colorbar = self.fig.colorbar(mappable, ax=ax, **kwargs)
        colorbar.ax.tick_params(labelsize=COLORBAR_TICK_SIZE)
        colorbar.ax.yaxis.label.set_size(COLORBAR_LABEL_SIZE)
        self.colorbars.append(colorbar)
        return colorbar

    def draw_trajectories(self) -> None:
        for idx, (ax, robot) in enumerate(zip(self.ax_trajectories, self.robots)):
            ax.clear()
            style_map_ax(ax, self.grid, self.source_xy, self.obstacles)
            ax.set_title(f"{robot.name} Trajectory and Gas", fontsize=TITLE_FONT_SIZE, fontweight="bold")

            end = self.history_end_index(robot)
            points = robot.positions[:end]
            gas = robot.gas[:end]
            color = ROBOT_COLORS[idx % len(ROBOT_COLORS)]

            if len(points) >= 2:
                ax.plot(
                    points[:, 0],
                    points[:, 1],
                    color=color,
                    linewidth=TRAJECTORY_GUIDE_LINE_WIDTH,
                    alpha=0.7,
                    zorder=2,
                )
                ax.add_collection(make_line_collection(points, gas, self.gas_norm, "viridis"))

            if len(points) > 0:
                ax.scatter(
                    points[:, 0],
                    points[:, 1],
                    c=gas,
                    cmap="viridis",
                    norm=self.gas_norm,
                    s=TRAJECTORY_SCATTER_SIZE,
                    edgecolors="#222222",
                    linewidths=0.35,
                    zorder=4,
                )
                ax.scatter(
                    points[0, 0],
                    points[0, 1],
                    marker="o",
                    s=START_MARKER_SIZE,
                    color=color,
                    edgecolors="black",
                    linewidths=0.9,
                    zorder=6,
                )
                ax.scatter(
                    points[-1, 0],
                    points[-1, 1],
                    marker="s",
                    s=CURRENT_MARKER_SIZE,
                    color=color,
                    edgecolors="black",
                    linewidths=0.9,
                    zorder=6,
                )

        handles = [
            Line2D(
                [0],
                [0],
                marker="o",
                color="w",
                markerfacecolor="#bbbbbb",
                markeredgecolor="black",
                markersize=LEGEND_MARKER_SIZE,
                label="Start",
            ),
            Line2D(
                [0],
                [0],
                marker="s",
                color="w",
                markerfacecolor="#bbbbbb",
                markeredgecolor="black",
                markersize=LEGEND_MARKER_SIZE,
                label="Current",
            ),
        ]
        for ax in self.ax_trajectories:
            ax.legend(handles=handles, loc="upper right", fontsize=LEGEND_FONT_SIZE, framealpha=0.9)

        mappable = plt.cm.ScalarMappable(norm=self.gas_norm, cmap="viridis")
        self.add_colorbar(mappable, ax=self.ax_trajectories, fraction=0.025, pad=0.02, label="Gas measurement")

    def draw_individual_beliefs(self) -> None:
        for idx, (ax, robot) in enumerate(zip(self.ax_beliefs, self.robots)):
            ax.clear()
            belief = reshape_map(row_at(robot.posterior, self.iteration), self.grid)
            mesh = ax.pcolormesh(
                self.grid.x_edges,
                self.grid.y_edges,
                normalize_map(belief),
                cmap=self.robot_cmaps[idx],
                vmin=0,
                vmax=1,
                shading="auto",
                rasterized=True,
                zorder=1,
            )
            style_map_ax(ax, self.grid, self.source_xy, self.obstacles)
            ax.set_title(f"{robot.name} Belief", fontsize=TITLE_FONT_SIZE, fontweight="bold")
            self.add_colorbar(mesh, ax=ax, fraction=0.046, pad=0.03, ticks=[0, 1])

    def draw_merged_belief(self) -> None:
        ax = self.ax_merged
        ax.clear()
        merged = self.merged_row()
        if merged is None:
            ax.text(0.5, 0.5, "No merged_pdf.csv", ha="center", va="center", transform=ax.transAxes)
            ax.set_axis_off()
            return

        merged_map = reshape_map(merged, self.grid)
        mesh = ax.pcolormesh(
            self.grid.x_edges,
            self.grid.y_edges,
            normalize_map(merged_map),
            cmap=self.merged_cmap,
            vmin=0,
            vmax=1,
            shading="auto",
            rasterized=True,
            zorder=1,
        )
        style_map_ax(ax, self.grid, self.source_xy, self.obstacles)
        ax.set_title("Merged Belief Map", fontsize=TITLE_FONT_SIZE, fontweight="bold")
        self.add_colorbar(mesh, ax=ax, fraction=0.046, pad=0.03, ticks=[0, 1])

    def draw_region_allocation(self) -> None:
        ax = self.ax_region
        ax.clear()
        allocation = self.region_allocation_row()
        if allocation is None:
            ax.text(0.5, 0.5, "No region_allocation.csv", ha="center", va="center", transform=ax.transAxes)
            ax.set_axis_off()
            ax.set_title("Region Allocation", fontsize=TITLE_FONT_SIZE, fontweight="bold")
            return

        allocation_map = reshape_map(np.rint(allocation).astype(int), self.grid)
        ax.pcolormesh(
            self.grid.x_edges,
            self.grid.y_edges,
            allocation_map,
            cmap=self.region_cmap,
            norm=self.region_norm,
            shading="auto",
            rasterized=True,
            zorder=1,
        )
        style_map_ax(ax, self.grid, self.source_xy, self.obstacles)
        ax.set_title("Region Allocation", fontsize=TITLE_FONT_SIZE, fontweight="bold")

        handles = [
            mpatches.Patch(facecolor="#f0f0f0", edgecolor="#777777", label="Unassigned")
        ]
        handles.extend(
            mpatches.Patch(
                facecolor=ROBOT_COLORS[idx % len(ROBOT_COLORS)],
                edgecolor="#333333",
                label=f"{idx + 1}: {robot.name}",
            )
            for idx, robot in enumerate(self.robots)
        )

        goal_handles: list[Line2D] = []
        for idx, robot in enumerate(self.robots):
            goal_xy = self.robot_goal_xy(robot)
            if goal_xy is None:
                continue

            color = ROBOT_COLORS[idx % len(ROBOT_COLORS)]
            ax.scatter(
                goal_xy[0],
                goal_xy[1],
                marker="D",
                s=GOAL_MARKER_SIZE,
                facecolor=color,
                edgecolor="black",
                linewidth=1.0,
                zorder=9,
            )
            goal_handles.append(
                Line2D(
                    [0],
                    [0],
                    marker="D",
                    linestyle="None",
                    markerfacecolor=color,
                    markeredgecolor="black",
                    markeredgewidth=1.0,
                    markersize=LEGEND_MARKER_SIZE,
                    label=f"{robot.name} goal",
                )
            )

        handles.extend(goal_handles)
        ax.legend(handles=handles, loc="upper right", fontsize=LEGEND_FONT_SIZE, framealpha=0.9)

    def draw_wind(self) -> None:
        self.ax_wind.clear()
        self.ax_uncertainty.clear()
        wind = self.wind_rows()
        if wind is None or self.wind_shape is None:
            for ax, title in (
                (self.ax_wind, "Estimated Wind Field"),
                (self.ax_uncertainty, "Wind Uncertainty"),
            ):
                ax.text(0.5, 0.5, "No wind_field_*.csv", ha="center", va="center", transform=ax.transAxes)
                ax.set_axis_off()
                ax.set_title(title, fontsize=TITLE_FONT_SIZE, fontweight="bold")
            return

        u_flat, v_flat, w_flat = wind
        u = reshape_wind(u_flat, self.wind_shape)
        v = reshape_wind(v_flat, self.wind_shape)
        w = reshape_wind(w_flat, self.wind_shape)
        ny, nx = self.wind_shape

        speed = np.sqrt(u * u + v * v)
        wind_mesh = self.ax_wind.imshow(
            speed,
            extent=[self.grid.x_min, self.grid.x_max, self.grid.y_min, self.grid.y_max],
            origin="lower",
            cmap="magma",
            aspect="auto",
            zorder=1,
        )

        x = np.linspace(self.grid.x_min, self.grid.x_max, nx)
        y = np.linspace(self.grid.y_min, self.grid.y_max, ny)
        xx, yy = np.meshgrid(x, y)
        step_x = max(1, nx // 18)
        step_y = max(1, ny // 10)
        self.ax_wind.quiver(
            xx[::step_y, ::step_x],
            yy[::step_y, ::step_x],
            u[::step_y, ::step_x],
            v[::step_y, ::step_x],
            color="white",
            pivot="mid",
            scale=None,
            width=0.004,
            zorder=4,
        )
        style_map_ax(self.ax_wind, self.grid, self.source_xy, self.obstacles)
        self.ax_wind.set_title("Estimated Wind Field", fontsize=TITLE_FONT_SIZE, fontweight="bold")
        self.add_colorbar(wind_mesh, ax=self.ax_wind, fraction=0.046, pad=0.03, label="Wind speed")

        if self.uncertainty_mode == "inverse":
            uncertainty = 1.0 / (1.0 + np.maximum(w, 0.0))
            label = "Inverse confidence"
        else:
            uncertainty = w
            label = "wind_field_w"

        uncert_mesh = self.ax_uncertainty.imshow(
            normalize_map(uncertainty),
            extent=[self.grid.x_min, self.grid.x_max, self.grid.y_min, self.grid.y_max],
            origin="lower",
            cmap="cividis",
            aspect="auto",
            zorder=1,
        )
        style_map_ax(self.ax_uncertainty, self.grid, self.source_xy, self.obstacles)
        self.ax_uncertainty.set_title("Wind Uncertainty", fontsize=TITLE_FONT_SIZE, fontweight="bold")
        self.add_colorbar(uncert_mesh, ax=self.ax_uncertainty, fraction=0.046, pad=0.03, label=label)

    def on_key(self, event) -> None:
        key = event.key
        if key in {"q", "escape"}:
            plt.close(self.fig)
            return
        if key in {"left", "backspace", "p"}:
            self.iteration = max(0, self.iteration - 1)
            self.draw()
            return
        if key == "home":
            self.iteration = 0
            self.draw()
            return
        if key == "end":
            self.iteration = self.n_iterations - 1
            self.draw()
            return
        if key in {"right", " ", "space", "n", "enter"} or key is not None:
            self.iteration = min(self.n_iterations - 1, self.iteration + 1)
            self.draw()

    def save_all(self) -> None:
        if self.save_dir is None:
            return

        self.save_dir.mkdir(parents=True, exist_ok=True)
        for iteration in range(self.n_iterations):
            self.iteration = iteration
            self.draw()
            out_path = self.save_dir / f"iteration_{iteration + 1:03d}.png"
            self.fig.savefig(out_path, dpi=180, facecolor="white")
            print(f"Saved {out_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Interactively plot multi-robot trajectories, belief maps, wind maps, and wind uncertainty."
    )
    parser.add_argument(
        "root_folder",
        nargs="?",
        type=Path,
        default=DEFAULT_ROOT,
        help="Experiment root folder containing k4/ and source_position.txt.",
    )
    parser.add_argument("--k-dir", default="k4", help="Robot log folder name under root_folder.")
    parser.add_argument(
        "--history-mode",
        choices=["prefix", "full"],
        default="prefix",
        help="Use progressive trajectory prefixes by iteration, or show the full trajectory on every iteration.",
    )
    parser.add_argument(
        "--uncertainty-mode",
        choices=["inverse", "raw"],
        default="inverse",
        help="Plot wind uncertainty as inverse confidence from wind_field_w.csv, or plot raw wind_field_w values.",
    )
    parser.add_argument(
        "--obstacle-file",
        type=Path,
        default=None,
        help="Optional obstacle file with one rectangle per line: x_min x_max y_min y_max.",
    )
    parser.add_argument("--save-dir", type=Path, default=None, help="Optional folder to save every iteration as PNG.")
    parser.add_argument("--no-show", action="store_true", help="Do not open the interactive window.")
    parser.add_argument(
        "--fig-scale",
        type=float,
        default=1.0,
        help="Scale the default figure size. Use values like 1.25 or 1.5 for larger plots.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    root = args.root_folder.expanduser().resolve()
    k_dir = root / args.k_dir
    if not k_dir.exists():
        raise FileNotFoundError(f"Cannot find robot log directory: {k_dir}")

    robot_dirs = discover_robot_dirs(k_dir)
    if not robot_dirs:
        raise ValueError(f"No robot log directories found under: {k_dir}")

    robots = [load_robot_log(folder) for folder in robot_dirs]
    pdf_cols = robots[0].posterior.shape[1]
    grid = parse_param_log(robot_dirs[0] / "param_log.csv", pdf_cols)

    wind_shape = None
    for robot in robots:
        if robot.wind_u is not None:
            wind_shape = infer_wind_shape(robot.wind_u.shape[1], grid)
            break

    source_xy = load_source_position(root)
    obstacle_file = args.obstacle_file
    if obstacle_file is None:
        candidate = root / "obstacle_list.txt"
        obstacle_file = candidate if candidate.exists() else None
    obstacles = load_obstacles(obstacle_file)

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 11,
            "axes.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )

    viewer = ExperimentViewer(
        robots,
        grid,
        wind_shape,
        source_xy,
        obstacles,
        history_mode=args.history_mode,
        uncertainty_mode=args.uncertainty_mode,
        fig_scale=args.fig_scale,
        save_dir=args.save_dir,
    )
    viewer.draw()
    viewer.save_all()

    print(
        "Controls: right/space/n/enter next, left/backspace/p previous, "
        "home/end jump, q/escape close."
    )
    print(f"Loaded {len(robots)} robots and {viewer.n_iterations} iterations from {k_dir}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
