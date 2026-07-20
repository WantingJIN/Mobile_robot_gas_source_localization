#!/usr/bin/env python3
"""
Interactive per-iteration belief-map and region-allocation viewer for multi-robot
GSL experiments.

Base layout per iteration (GridSpec 2×6, or 3×6 with wind):
  Row 0 (each 2/6 cols): [R1 Indiv. Belief] [R2 Indiv. Belief] [R3 Indiv. Belief]
  Row 1 (each 3/6 cols): [Merged Belief   ] [Region Allocation]
  Row 2, if wind_info == 1: [Wind Field] [Wind Uncertainty]

Both rows span identical total width. Top plots are each 2/3 the width of bottom plots.

Files used (all relative to exp_folder/<robot_id>/):
  posterior_pdf_pre_merge.csv   : rows=FSM cycles, cols=400 cell probabilities (per robot)
  posterior_pdf_3D.csv          : rows=FSM cycles, cols=400 merged belief intensities (R101 only)
  posterior_pdf_3D_cluster.csv  : rows=FSM cycles, cols=2 goal_xy + 400 cluster IDs (R101 only)
  goal_position.csv             : cols=[cycle, goal_x, goal_y, ...]
  measurement_history.csv       : cols=[step, concentration, norm_conc, FSM_iteration]
  position_history.csv          : cols=[step, x, y, heading] — rows align 1:1 with above
  source_pos.csv                : "source_position x y z"
  wind_field_u/v/w.csv          : optional online wind estimate and uncertainty

If a base folder is used, a small window lets you choose the experiment
subfolder to open. Press space to advance to the next iteration,
left/backspace to go back, and q to quit. Use --save to save plots; by default
nothing is written.
"""

from __future__ import annotations

import argparse
from copy import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap, PowerNorm
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D

# ── Configuration ────────────────────────────────────────────────────────────
DEFAULT_BASE_PATH = Path(
    '/home/wjin/Workspace/Multi-robot_GSL/Results/multi_test/revert_back_region_allocation/3rob_ADE_wind_online/10'
)
ROBOT_IDS   = (101, 102, 103)
ROBOT_NAMES = ['Robot 1', 'Robot 2', 'Robot 3']
REFERENCE_ROBOT = 101
OBSTACLE_WIND_SPEED = '0.75'
BELIEF_GAMMA = 0.35
ZERO_BELIEF_COLOR = '#000000'
SOURCE_COLOR = '#d62728'
DEFAULT_WIND_GRID_SHAPE = (100, 40)  # Nx, Ny, matching show_info_map.py

# Grid setup (matching MATLAB script)
N_COLS  = 20
N_ROWS  = 20
X_EDGES = np.linspace(3, 12.5, N_COLS + 1)
Y_EDGES = np.linspace(0.16, 3.8,  N_ROWS + 1)

X_LIM = (2.9, 13.0)
Y_LIM = (0.0,  4.0)

# ── Color scheme ─────────────────────────────────────────────────────────────
def _with_zero_belief_color(cmap):
    cmap = copy(cmap)
    cmap.set_bad(ZERO_BELIEF_COLOR)
    return cmap


# Full robot colors (used for measurement markers and belief map high end)
ROBOT_COLORS = ['#1a6faf', '#d95f02', '#1b7837']   # blue, orange, green

# Per-robot colormaps: white (low concentration) → robot color (high).
ROBOT_CMAPS = [
    LinearSegmentedColormap.from_list('', ['#ffffff', c], N=256)
    for c in ROBOT_COLORS
]
# Belief heatmaps mask exact zero values to black, then use a nonlinear scale
# for positive values to make small probabilities easier to distinguish.
ROBOT_BELIEF_CMAPS = [_with_zero_belief_color(cmap) for cmap in ROBOT_CMAPS]

# Merged belief: white → dark charcoal
MERGED_CMAP = _with_zero_belief_color(
    LinearSegmentedColormap.from_list('', ['#ffffff', '#1e2d40'], N=256)
)
WIND_CMAP = 'turbo'

# Cluster region fill colors (lighter than robot colors for region background)
CLUSTER_BASE = {
    0: np.array([0.92, 0.92, 0.92]),
    1: np.array([0.40, 0.66, 0.84]),
    2: np.array([0.99, 0.55, 0.23]),
    3: np.array([0.40, 0.76, 0.44]),
}

DARK_GRAY          = '#3a3a3a'
MIN_VISIBLE_INTENSITY = 0.15


@dataclass
class WindData:
    u: np.ndarray
    v: np.ndarray
    uncertainty: np.ndarray
    nx: int
    ny: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass
class ExperimentData:
    base_path: Path
    exp_folder: Path
    robot_ids: tuple[int, ...]
    reference_robot: int
    map_nbr: int
    obstacles: list[tuple[float, float, float, float]]
    pre_merge: list[np.ndarray]
    merged: np.ndarray
    cluster_maps: np.ndarray
    goal_positions: list[np.ndarray]
    meas_history: list[np.ndarray]
    pos_history: list[np.ndarray]
    meas_global_max: list[float]
    source_xy: tuple[float, float]
    wind: WindData | None


# ── Matplotlib style ──────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':       'DejaVu Sans',
    'font.size':         14,
    'axes.linewidth':    0.8,
    'axes.spines.top':   False,
    'axes.spines.right': False,
    'xtick.major.width': 0.8,
    'ytick.major.width': 0.8,
    'xtick.direction':   'out',
    'ytick.direction':   'out',
})


# ── Experiment selection and data loading ────────────────────────────────────
def workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


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


def param_as_int(params: dict[str, float | list[float]], key: str, default: int = 0) -> int:
    value = params.get(key, default)
    if isinstance(value, list):
        value = value[0] if value else default
    return int(value)


def load_obstacles(map_nbr: int, root: Path) -> list[tuple[float, float, float, float]]:
    obstacle_path = (
        root
        / 'openfoam'
        / 'test_maps'
        / f'map{map_nbr}'
        / f'map{map_nbr}_wind{OBSTACLE_WIND_SPEED}'
        / f'{map_nbr}_obstacles.txt'
    )
    obstacles: list[tuple[float, float, float, float]] = []
    if not obstacle_path.exists():
        print(f'Warning: obstacle file not found: {obstacle_path}')
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
        obstacles.append((x1, y1, x2 - x1, y2 - y1))
    return obstacles


def has_robot_folders(path: Path, robot_ids: Iterable[int]) -> bool:
    return all((path / str(robot_id)).is_dir() for robot_id in robot_ids)


def has_plot_files(exp_folder: Path, robot_ids: tuple[int, ...], reference_robot: int) -> bool:
    ref_path = exp_folder / str(reference_robot)
    required = [
        ref_path / 'param_log.csv',
        ref_path / 'posterior_pdf_3D.csv',
        ref_path / 'posterior_pdf_3D_cluster.csv',
        ref_path / 'source_pos.csv',
    ]
    for robot_id in robot_ids:
        robot_path = exp_folder / str(robot_id)
        required.extend([
            robot_path / 'posterior_pdf_pre_merge.csv',
            robot_path / 'goal_position.csv',
            robot_path / 'measurement_history.csv',
            robot_path / 'position_history.csv',
        ])
    return all(path.exists() for path in required)


def experiment_candidates(
    base_path: Path,
    robot_ids: tuple[int, ...],
    reference_robot: int,
) -> list[Path]:
    return [
        child
        for child in sorted(base_path.iterdir())
        if child.is_dir()
        and not child.name.startswith('.')
        and has_robot_folders(child, robot_ids)
        and has_plot_files(child, robot_ids, reference_robot)
    ]


def choose_experiment_folder_gui(base_path: Path, candidates: list[Path]) -> Path | None:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError as exc:
        print(f'GUI folder picker unavailable ({exc}); using terminal prompt.')
        return None

    root = None
    try:
        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        selected = filedialog.askdirectory(
            parent=root,
            initialdir=str(base_path),
            title='Select experiment subfolder',
            mustexist=True,
        )
    except Exception as exc:
        print(f'GUI folder picker unavailable ({exc}); using terminal prompt.')
        return None
    finally:
        if root is not None:
            root.destroy()

    if not selected:
        raise SystemExit('No experiment selected.')

    selected_path = Path(selected).expanduser().resolve()
    valid_paths = {candidate.resolve(): candidate for candidate in candidates}
    if selected_path not in valid_paths:
        raise FileNotFoundError(
            f'Selected folder is not a valid experiment subfolder under {base_path}: {selected_path}'
        )
    return valid_paths[selected_path]


def choose_experiment_folder(base_path: Path, candidates: list[Path], use_gui: bool = True) -> Path:
    if use_gui:
        selected = choose_experiment_folder_gui(base_path, candidates)
        if selected is not None:
            return selected

    print(f'Available experiment folders under {base_path}:')
    for idx, candidate in enumerate(candidates):
        print(f'  [{idx}] {candidate.name}')

    while True:
        try:
            choice = input('Choose experiment folder index (q to quit): ').strip()
        except EOFError as exc:
            raise SystemExit(
                'No experiment selected. Pass an experiment folder path or use --experiment-index.'
            ) from exc

        if choice.lower() == 'q':
            raise SystemExit('No experiment selected.')
        try:
            idx = int(choice)
        except ValueError:
            print('Please enter a numeric index.')
            continue
        if 0 <= idx < len(candidates):
            return candidates[idx]
        print(f'Index out of range. Choose 0 to {len(candidates) - 1}.')


def resolve_experiment_folder(
    selected_path: Path | None,
    robot_ids: tuple[int, ...],
    reference_robot: int,
    experiment_index: int | None,
    use_gui: bool = True,
) -> tuple[Path, Path]:
    selected_path = (selected_path or DEFAULT_BASE_PATH).expanduser().resolve()

    if (selected_path / 'param_log.csv').exists() and selected_path.name in {
        str(robot_id) for robot_id in robot_ids
    }:
        exp_folder = selected_path.parent
        return exp_folder.parent, exp_folder

    if has_robot_folders(selected_path, robot_ids):
        return selected_path.parent, selected_path

    candidates = experiment_candidates(selected_path, robot_ids, reference_robot)
    if not candidates:
        raise FileNotFoundError(f'No experiment subfolder found in {selected_path}')
    if experiment_index is None:
        return selected_path, choose_experiment_folder(selected_path, candidates, use_gui)
    if experiment_index < 0 or experiment_index >= len(candidates):
        raise IndexError(
            f'Experiment index {experiment_index} out of range; found {len(candidates)} subfolders'
        )
    return selected_path, candidates[experiment_index]


def load_csv(path: Path) -> np.ndarray:
    return np.atleast_2d(np.loadtxt(path, delimiter=','))


def wind_info_enabled(params: dict[str, float | list[float]]) -> bool:
    return (
        param_as_int(params, 'wind_info', 0) == 1
        or param_as_int(params, 'windInfo', 0) == 1
    )


def infer_wind_shape(n_values: int) -> tuple[int, int]:
    nx, ny = DEFAULT_WIND_GRID_SHAPE
    if nx * ny == n_values:
        return nx, ny
    side = int(round(np.sqrt(n_values)))
    if side * side == n_values:
        return side, side
    raise ValueError(
        f'Could not infer wind grid shape for {n_values} values; '
        f'expected {nx * ny} from {DEFAULT_WIND_GRID_SHAPE}'
    )


def load_wind_data(reference_path: Path, params: dict[str, float | list[float]]) -> WindData | None:
    if not wind_info_enabled(params):
        return None

    wind_paths = {
        'u': reference_path / 'wind_field_u.csv',
        'v': reference_path / 'wind_field_v.csv',
        'uncertainty': reference_path / 'wind_field_w.csv',
    }
    missing = [str(path) for path in wind_paths.values() if not path.exists()]
    if missing:
        print('wind_info=1 but wind files are missing; skipping wind panels:')
        for path in missing:
            print(f'  {path}')
        return None

    u = load_csv(wind_paths['u'])
    v = load_csv(wind_paths['v'])
    uncertainty = load_csv(wind_paths['uncertainty'])
    if u.shape != v.shape or u.shape != uncertainty.shape:
        raise ValueError(
            'Wind field CSV shapes do not match: '
            f'u={u.shape}, v={v.shape}, uncertainty={uncertainty.shape}'
        )

    nx, ny = infer_wind_shape(u.shape[1])
    min_maxs = params.get('min_maxs')
    if isinstance(min_maxs, list) and len(min_maxs) >= 6:
        x_min, x_max = float(min_maxs[2]), float(min_maxs[3])
        y_min, y_max = float(min_maxs[4]), float(min_maxs[5])
    else:
        x_min, x_max = X_LIM
        y_min, y_max = Y_LIM

    return WindData(
        u=u,
        v=v,
        uncertainty=uncertainty,
        nx=nx,
        ny=ny,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
    )


def load_data(base_path: Path, exp_folder: Path, robot_ids: tuple[int, ...], reference_robot: int) -> ExperimentData:
    reference_path = exp_folder / str(reference_robot)
    params = parse_param_log(reference_path / 'param_log.csv')
    map_nbr = param_as_int(params, 'map_nbr')
    obstacles = load_obstacles(map_nbr, workspace_root()) if map_nbr else []
    wind = load_wind_data(reference_path, params)

    pre_merge = []
    for rid in robot_ids:
        path = exp_folder / str(rid) / 'posterior_pdf_pre_merge.csv'
        pre_merge.append(load_csv(path))

    merged = load_csv(reference_path / 'posterior_pdf_3D.csv')

    cluster_raw  = load_csv(reference_path / 'posterior_pdf_3D_cluster.csv')
    cluster_maps = cluster_raw[:, 2:].astype(int)

    goal_positions = []
    for rid in robot_ids:
        path = exp_folder / str(rid) / 'goal_position.csv'
        g = load_csv(path)
        goal_positions.append(g[:, 1:3])

    # Measurement history: [step, concentration, norm_conc, FSM_iteration]
    # Position history:    [step, x, y, heading]   — same row count, 1:1 alignment
    meas_history = []
    pos_history  = []
    for rid in robot_ids:
        meas_history.append(load_csv(exp_folder / str(rid) / 'measurement_history.csv'))
        pos_history.append(load_csv(exp_folder / str(rid) / 'position_history.csv'))

    # Per-robot global max of raw concentration (col 1) for consistent normalisation
    meas_global_max = [float(m[:, 1].max()) if m.shape[1] > 1 else 0.0 for m in meas_history]

    with (reference_path / 'source_pos.csv').open() as f:
        tokens = f.read().split()
    source_xy = (float(tokens[1]), float(tokens[2]))

    return ExperimentData(
        base_path=base_path,
        exp_folder=exp_folder,
        robot_ids=robot_ids,
        reference_robot=reference_robot,
        map_nbr=map_nbr,
        obstacles=obstacles,
        pre_merge=pre_merge,
        merged=merged,
        cluster_maps=cluster_maps,
        goal_positions=goal_positions,
        meas_history=meas_history,
        pos_history=pos_history,
        meas_global_max=meas_global_max,
        source_xy=source_xy,
        wind=wind,
    )


# ── Shared helpers ────────────────────────────────────────────────────────────
def _normalize(arr):
    mn, mx = arr.min(), arr.max()
    return (arr - mn) / (mx - mn) if mx > mn else np.zeros_like(arr)


def _belief_display_grid(arr):
    raw = np.asarray(arr, dtype=float)
    mx = np.nanmax(raw)
    if np.isfinite(mx) and mx > 0:
        display = raw / mx
    else:
        display = np.zeros_like(raw)
    masked = np.ma.masked_where(raw == 0, display)
    return masked.reshape(N_ROWS, N_COLS)


def _wind_matrix(row: np.ndarray, wind: WindData) -> np.ndarray:
    return np.reshape(row[: wind.nx * wind.ny], (wind.nx, wind.ny), order='F')


def _wind_row(data: np.ndarray, it: int) -> np.ndarray:
    return data[min(it, len(data) - 1)]


def _style_ax(ax, source_xy, obstacles):
    """Obstacles, source marker, axis limits, y-ticks."""
    for ox, oy, ow, oh in obstacles:
        ax.add_patch(mpatches.Rectangle(
            (ox, oy), ow, oh, facecolor=DARK_GRAY, edgecolor='none', zorder=3))

    ax.plot(*source_xy, '*', markersize=9,
            markeredgecolor='white', markerfacecolor=SOURCE_COLOR,
            markeredgewidth=0.5, zorder=8)

    ax.set_xlim(*X_LIM)
    ax.set_ylim(*Y_LIM)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', fontsize=13)
    ax.set_yticks([0, 4])
    ax.set_yticklabels(['0 m', '4 m'], fontsize=12)
    ax.tick_params(axis='x', labelsize=12)


def _style_wind_ax(ax, source_xy, obstacles, wind: WindData):
    _style_ax(ax, source_xy, obstacles)
    ax.set_xlim(wind.x_min, wind.x_max)
    ax.set_ylim(wind.y_min, wind.y_max)


def _add_colorbar(ax, mesh):
    """Thin vertical colorbar on the right of ax."""
    cb = ax.figure.colorbar(
        mesh, ax=ax,
        fraction=0.046, pad=0.03, aspect=22,
        ticks=[0, 1],
    )
    cb.ax.set_yticklabels(['Low', 'High'], fontsize=11)
    cb.outline.set_linewidth(0.5)
    cb.ax.tick_params(length=2, width=0.5)


def _plot_measurements(ax, meas, pos, global_max, iteration_limit, r_idx):
    """Scatter gas measurements up to FSM iteration `iteration_limit`.

    Markers are coloured by raw concentration (col 1) normalised to [0, global_max],
    using the robot's individual colormap (white = no gas, full colour = peak gas).
    Zero readings are still drawn so the measurement path starts at the initial
    robot position even before gas is detected.
    """
    mask    = meas[:, 3] <= iteration_limit      # rows up to this iteration
    conc    = meas[mask, 1]                       # raw concentration
    xy      = pos[mask, 1:3]                      # x, y positions

    if len(xy) == 0:
        return

    if iteration_limit >= 1 and len(xy) > 1:
        ax.plot(
            xy[:, 0], xy[:, 1],
            color=ROBOT_COLORS[r_idx],
            linewidth=1.1,
            alpha=0.42,
            zorder=4,
        )

    c_norm = np.clip(conc / (global_max + 1e-10), 0, 1)

    ax.scatter(xy[:, 0], xy[:, 1],
               c=c_norm, cmap=ROBOT_CMAPS[r_idx],
               vmin=0, vmax=1,
               s=18, zorder=5,
               edgecolors='#333333', linewidths=0.25,
               alpha=0.85)


# ── Per-panel plotting ────────────────────────────────────────────────────────
def plot_individual_belief(ax, belief_1d, r_idx, source_xy, obstacles,
                           meas, pos, global_max, iteration_limit):
    """Belief heatmap (white→robot colour) with measurement scatter overlay."""
    mesh = ax.pcolormesh(X_EDGES, Y_EDGES, _belief_display_grid(belief_1d),
                         cmap=ROBOT_BELIEF_CMAPS[r_idx],
                         norm=PowerNorm(gamma=BELIEF_GAMMA, vmin=0, vmax=1),
                         zorder=1, rasterized=True)
    _style_ax(ax, source_xy, obstacles)
    _plot_measurements(ax, meas, pos, global_max, iteration_limit, r_idx)
    trajectory_handle = Line2D([0], [0], color=ROBOT_COLORS[r_idx],
                               linewidth=1.1, alpha=0.42, label='Trajectory')
    sensor_handle = Line2D([0], [0], marker='o', color='w',
                           markerfacecolor=ROBOT_COLORS[r_idx],
                           markeredgecolor='#333333', markeredgewidth=0.3,
                           markersize=5, label='Sensor output')
    handles = [sensor_handle]
    if iteration_limit >= 1:
        handles.insert(0, trajectory_handle)
    ax.legend(handles=handles, loc='upper left', fontsize=11,
              framealpha=0.85, handlelength=1.0, borderpad=0.4)
    _add_colorbar(ax, mesh)
    ax.set_title(f'{ROBOT_NAMES[r_idx]} — Individual Belief',
                 fontsize=13, fontweight='bold', pad=4)


def plot_merged_belief(ax, intensity_1d, source_xy, obstacles):
    """Merged belief heatmap (white → dark charcoal), no measurement overlay."""
    mesh = ax.pcolormesh(X_EDGES, Y_EDGES, _belief_display_grid(intensity_1d),
                         cmap=MERGED_CMAP,
                         norm=PowerNorm(gamma=BELIEF_GAMMA, vmin=0, vmax=1),
                         zorder=1, rasterized=True)
    _style_ax(ax, source_xy, obstacles)
    _add_colorbar(ax, mesh)
    ax.set_title('Merged Belief Map', fontsize=13, fontweight='bold', pad=4)


def plot_cluster_allocation(ax, cluster_1d, intensity_1d, goals, source_xy, obstacles):
    """Region allocation with intensity-blended cluster colours. Goals shown here only."""
    norm_int = _normalize(intensity_1d)

    img = np.ones((N_ROWS, N_COLS, 3))
    for ri in range(N_ROWS):
        for ci in range(N_COLS):
            idx       = ri * N_COLS + ci
            cid       = cluster_1d[idx]
            intensity = float(np.clip(norm_int[idx], 0, 1))
            base      = CLUSTER_BASE.get(cid, CLUSTER_BASE[0])
            alpha     = MIN_VISIBLE_INTENSITY + (1 - MIN_VISIBLE_INTENSITY) * intensity
            img[ri, ci] = base * alpha + np.ones(3) * (1 - alpha)

    ax.imshow(img, origin='lower', zorder=1, aspect='auto', rasterized=True,
              extent=[X_EDGES[0], X_EDGES[-1], Y_EDGES[0], Y_EDGES[-1]])
    _style_ax(ax, source_xy, obstacles)

    for r_idx, gxy in enumerate(goals):
        if gxy is not None:
            ax.plot(*gxy, 'D', markersize=7, linewidth=0,
                    color=ROBOT_COLORS[r_idx], markeredgecolor='k',
                    markeredgewidth=0.6, zorder=7)

    source_handle = ax.plot([], [], '*', markersize=8,
                            markeredgecolor='white', markerfacecolor=SOURCE_COLOR,
                            markeredgewidth=0.5, label='True source')[0]
    goal_handles = [
        Line2D([0], [0], marker='D', color='w',
               markerfacecolor=ROBOT_COLORS[r], markeredgecolor='k',
               markeredgewidth=0.6, markersize=7,
               label=f'{ROBOT_NAMES[r]} next goal')
        for r in range(3)
    ]
    patches = [
        source_handle,
        *goal_handles,
        mpatches.Patch(facecolor=CLUSTER_BASE[1], edgecolor='#555',
                       linewidth=0.5, label=f'{ROBOT_NAMES[0]} region'),
        mpatches.Patch(facecolor=CLUSTER_BASE[2], edgecolor='#555',
                       linewidth=0.5, label=f'{ROBOT_NAMES[1]} region'),
        mpatches.Patch(facecolor=CLUSTER_BASE[3], edgecolor='#555',
                       linewidth=0.5, label=f'{ROBOT_NAMES[2]} region'),
        mpatches.Patch(facecolor=CLUSTER_BASE[0], edgecolor='#555',
                       linewidth=0.5, label='Unassigned'),
    ]
    ax.legend(handles=patches, loc='upper left', bbox_to_anchor=(1.01, 1),
              borderaxespad=0, fontsize=11,
              framealpha=0.85, handlelength=1.0, borderpad=0.4)
    ax.set_title('Region Allocation', fontsize=13, fontweight='bold', pad=4)


def plot_wind_field(ax, wind: WindData, it: int, source_xy, obstacles):
    """Wind speed heatmap with direction arrows."""
    u = _wind_matrix(_wind_row(wind.u, it), wind)
    v = _wind_matrix(_wind_row(wind.v, it), wind)
    speed = np.sqrt(u**2 + v**2).T

    im = ax.imshow(
        speed,
        extent=[wind.x_min, wind.x_max, wind.y_min, wind.y_max],
        origin='lower',
        interpolation='nearest',
        cmap=WIND_CMAP,
        zorder=1,
    )

    ds = max(1, min(wind.nx, wind.ny) // 12)
    x = np.linspace(wind.x_min, wind.x_max, wind.nx)
    y = np.linspace(wind.y_min, wind.y_max, wind.ny)
    xx, yy = np.meshgrid(x, y)
    ax.quiver(
        xx[::ds, ::ds],
        yy[::ds, ::ds],
        u.T[::ds, ::ds],
        v.T[::ds, ::ds],
        color='black',
        linewidth=0.45,
        scale=None,
        scale_units='xy',
        angles='xy',
        zorder=4,
    )
    _style_wind_ax(ax, source_xy, obstacles, wind)
    cb = ax.figure.colorbar(im, ax=ax, fraction=0.046, pad=0.03, aspect=22)
    cb.set_label('Speed')
    ax.set_title('Wind Field', fontsize=13, fontweight='bold', pad=4)


def plot_wind_uncertainty(ax, wind: WindData, it: int, source_xy, obstacles):
    """Wind estimation uncertainty heatmap."""
    uncertainty = _wind_matrix(_wind_row(wind.uncertainty, it), wind).T
    im = ax.imshow(
        uncertainty,
        extent=[wind.x_min, wind.x_max, wind.y_min, wind.y_max],
        origin='lower',
        interpolation='nearest',
        cmap=WIND_CMAP,
        vmin=0,
        zorder=1,
    )
    _style_wind_ax(ax, source_xy, obstacles, wind)
    cb = ax.figure.colorbar(im, ax=ax, fraction=0.046, pad=0.03, aspect=22)
    cb.set_label('Uncertainty')
    ax.set_title('Wind Uncertainty', fontsize=13, fontweight='bold', pad=4)


# ── Main ─────────────────────────────────────────────────────────────────────
def iteration_count(data: ExperimentData, max_iters: int | None = None) -> int:
    counts = [
        data.merged.shape[0],
        *(arr.shape[0] for arr in data.pre_merge),
    ]
    n_iter = min(counts)
    if max_iters is not None:
        n_iter = min(n_iter, max_iters)
    return n_iter


def clamped_goal(gps: np.ndarray, it: int) -> tuple[float, float] | None:
    if len(gps) == 0:
        return None
    # goal_position row 1 corresponds to trajectory iteration 1; once the
    # sequence ends, keep showing the final goal while belief maps continue.
    row = min(it + 1, len(gps) - 1)
    return gps[row, 0], gps[row, 1]


def draw_iteration(fig: plt.Figure, data: ExperimentData, it: int, n_iter: int) -> None:
    # Belief maps, measurement_history and position_history use 0-based FSM iterations.
    iteration_limit = it
    cluster_idx = min(it, data.cluster_maps.shape[0] - 1)

    fig.clear()
    n_rows = 3 if data.wind is not None else 2
    gs  = GridSpec(n_rows, 6, figure=fig, hspace=0.45, wspace=0.35)

    ax_r1  = fig.add_subplot(gs[0, 0:2])
    ax_r2  = fig.add_subplot(gs[0, 2:4])
    ax_r3  = fig.add_subplot(gs[0, 4:6])
    ax_mrg = fig.add_subplot(gs[1, 0:3])
    ax_cls = fig.add_subplot(gs[1, 3:6])
    if data.wind is not None:
        ax_wind = fig.add_subplot(gs[2, 0:3])
        ax_wunc = fig.add_subplot(gs[2, 3:6])

    goals = []
    for r_idx in range(3):
        goals.append(clamped_goal(data.goal_positions[r_idx], it))

    # Row 0: individual belief + measurements
    for r_idx, ax in enumerate([ax_r1, ax_r2, ax_r3]):
        plot_individual_belief(
            ax, data.pre_merge[r_idx][it], r_idx, data.source_xy, data.obstacles,
            data.meas_history[r_idx], data.pos_history[r_idx],
            data.meas_global_max[r_idx], iteration_limit,
        )

    # Row 1
    plot_merged_belief(ax_mrg, data.merged[it], data.source_xy, data.obstacles)
    plot_cluster_allocation(
        ax_cls, data.cluster_maps[cluster_idx], data.merged[it], goals, data.source_xy, data.obstacles
    )
    if data.wind is not None:
        plot_wind_field(ax_wind, data.wind, it, data.source_xy, data.obstacles)
        plot_wind_uncertainty(ax_wunc, data.wind, it, data.source_xy, data.obstacles)

    fig.suptitle(
        f'{data.exp_folder.name} | iteration {it} ({it + 1}/{n_iter}) | map {data.map_nbr}',
        fontsize=14,
        fontweight='bold',
    )


def figure_size(data: ExperimentData) -> tuple[float, float]:
    return (15, 9.2) if data.wind is not None else (15, 6)


def save_iteration(fig: plt.Figure, output_dir: Path, iteration_idx: int, file_format: str, dpi: int) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f'iteration_{iteration_idx:03d}.{file_format}'
    fig.savefig(out_path, bbox_inches='tight', facecolor='white', edgecolor='none', dpi=dpi)
    return out_path


def interactive_backend_available() -> bool:
    backend = plt.get_backend().lower()
    return backend not in {'agg', 'cairo', 'pdf', 'pgf', 'ps', 'svg', 'template'} and 'inline' not in backend


def save_all_iterations(
    data: ExperimentData,
    output_dir: Path,
    file_format: str,
    dpi: int,
    n_iter: int,
) -> None:
    fig = plt.figure(figsize=figure_size(data), facecolor='white')
    print(f'Saving {n_iter} plots to: {output_dir}')
    for idx in range(n_iter):
        draw_iteration(fig, data, idx, n_iter)
        out_path = save_iteration(fig, output_dir, idx, file_format, dpi)
        print(f'  [{idx + 1:2d}/{n_iter}] {out_path.name}')
    plt.close(fig)
    print(f'Plots saved to: {output_dir}')


def run(data: ExperimentData, args: argparse.Namespace) -> None:
    n_iter = iteration_count(data, args.max_iters)
    if n_iter <= 0:
        raise ValueError('No iterations to display')

    output_dir = args.output_dir or (data.exp_folder / 'output_plots')
    if args.save:
        save_all_iterations(data, output_dir, args.format, args.dpi, n_iter)

    if not interactive_backend_available():
        if args.save:
            print(
                f'Matplotlib backend {plt.get_backend()} is non-interactive; '
                'skipping the interactive viewer.'
            )
            return
        raise SystemExit(
            f'Matplotlib backend {plt.get_backend()} is non-interactive. '
            'Use a GUI backend to view interactively, or rerun with --save.'
        )

    fig = plt.figure(figsize=figure_size(data), facecolor='white')

    state = {'quit': False, 'step': 1}

    def on_key(event) -> None:
        if event.key in {'q', 'escape'}:
            state['quit'] = True
            event.canvas.stop_event_loop()
        elif event.key in {' ', 'space'}:
            state['step'] = 1
            event.canvas.stop_event_loop()
        elif event.key in {'left', 'backspace'}:
            state['step'] = -1
            event.canvas.stop_event_loop()

    def on_close(event) -> None:
        state['quit'] = True
        event.canvas.stop_event_loop()

    fig.canvas.mpl_connect('key_press_event', on_key)
    fig.canvas.mpl_connect('close_event', on_close)
    plt.show(block=False)
    print('Press space for next iteration, left/backspace for previous, q to quit.')

    idx = 0
    while not state['quit']:
        draw_iteration(fig, data, idx, n_iter)

        fig.canvas.draw()
        fig.canvas.flush_events()
        state['step'] = 1
        fig.canvas.start_event_loop(0)

        if state['quit']:
            break
        next_idx = idx + state['step']
        if next_idx < 0:
            idx = 0
        elif next_idx >= n_iter:
            break
        else:
            idx = next_idx

    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Step through belief maps and region allocation plots.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        'path',
        nargs='?',
        type=Path,
        default=None,
        help='Base folder, experiment folder, or robot folder. If omitted, DEFAULT_BASE_PATH is used.',
    )
    parser.add_argument(
        '--experiment-index',
        type=int,
        default=None,
        help='Subfolder index when path is a base folder. If omitted, a picker window is shown.',
    )
    parser.add_argument(
        '--no-gui',
        action='store_true',
        help='Use the terminal prompt instead of the experiment folder picker window.',
    )
    parser.add_argument('--max-iters', type=int, default=None, help='Limit displayed iterations.')
    parser.add_argument('--save', action='store_true', help='Save all selected plots. Default is display only.')
    parser.add_argument('--output-dir', type=Path, default=None, help='Output directory for --save.')
    parser.add_argument('--format', choices=('pdf', 'png'), default='pdf', help='Saved plot format for --save.')
    parser.add_argument('--dpi', type=int, default=150, help='Saved plot DPI.')
    args = parser.parse_args()
    args.gui_picker = not args.no_gui
    if args.output_dir is not None:
        args.output_dir = args.output_dir.expanduser().resolve()
    return args


def main() -> None:
    args = parse_args()
    base_path, exp_folder = resolve_experiment_folder(
        args.path,
        ROBOT_IDS,
        REFERENCE_ROBOT,
        args.experiment_index,
        args.gui_picker,
    )
    data = load_data(base_path, exp_folder, ROBOT_IDS, REFERENCE_ROBOT)
    print(f'Experiment folder: {data.exp_folder}')
    print(f'Reference robot: {data.reference_robot}')
    print(f'map_nbr from param_log: {data.map_nbr}')
    print(f'Loaded obstacles: {len(data.obstacles)}')
    if data.wind is not None:
        print(f'Loaded wind frames: {len(data.wind.u)} ({data.wind.nx}x{data.wind.ny})')
    print(f'Displayed iterations: {iteration_count(data, args.max_iters)}')
    run(data, args)


if __name__ == '__main__':
    main()
