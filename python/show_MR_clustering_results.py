#!/usr/bin/env python3
"""
Interactive clustering-result viewer for multi-robot GSL experiments.

For each clustering iteration this shows:
  1. posterior_pdf_3D_cluster.csv with per-robot cluster colours and each
     robot's trajectory segment for that iteration.
  2. posterior_pdf_3D.csv as the belief map for the same iteration.

If a base folder is used, a small window lets you choose the experiment
subfolder to open. Press any key to advance to the next iteration,
left/backspace to go back, and q to quit.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_BASE_PATH = Path(
    "/home/wjin/Workspace/Multi-robot_GSL/Results/multi_test/PDE_belief_cluster_new_region_allocation/10"
)
DEFAULT_ROBOT_IDS = (101, 102, 103)
REFERENCE_ROBOT = 101

WALL_THICKNESS = 0.1
WORLD_X_LIMITS = (-WALL_THICKNESS, 16.0 + WALL_THICKNESS)
WORLD_Y_LIMITS = (-WALL_THICKNESS, 4.0 + WALL_THICKNESS)

CELL_X_MIN = 3.0
CELL_X_MAX = 12.5
CELL_Y_MIN = 0.16
CELL_Y_MAX = 3.8

AX_FACE = "#f7f8fb"
GRID_COLOR = "#d9dee7"
SPINE_COLOR = "#9aa4b2"
TEXT_COLOR = "#22272e"
DARK_GRAY = "#383d45"
SOURCE_COLOR = "black"

ROBOT_COLORS = {
    101: "#d95f02",
    102: "#1f77b4",
    103: "#2ca25f",
}
DEFAULT_TRAJ_COLORS = ["#d95f02", "#1f77b4", "#2ca25f", "#9467bd", "#8c564b"]

CLUSTER_CMAP = mcolors.ListedColormap(["#ffffff", "#d95f02", "#1f77b4", "#2ca25f"])
CLUSTER_NORM = mcolors.BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], CLUSTER_CMAP.N)
PDF_CMAP = mcolors.LinearSegmentedColormap.from_list(
    "belief_pdf",
    ["#ffffff", "#fff7bc", "#fec44f", "#fb6a4a", "#cb181d", "#67000d"],
)


@dataclass
class ExperimentData:
    base_path: Path
    exp_folder: Path
    robot_ids: tuple[int, ...]
    reference_robot: int
    map_nbr: int
    rows: int
    cols: int
    obstacles: list[tuple[float, float, float, float]]
    cluster_maps: np.ndarray
    pdf_maps: np.ndarray
    trajectories: dict[int, np.ndarray]
    source_xy: tuple[float, float]


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
            obstacles.append(tuple(float(part) for part in parts[:4]))
        except ValueError:
            continue
    return obstacles


def has_robot_folders(path: Path, robot_ids: Iterable[int]) -> bool:
    return all((path / str(robot_id)).is_dir() for robot_id in robot_ids)


def experiment_candidates(base_path: Path, robot_ids: tuple[int, ...]) -> list[Path]:
    return [
        child
        for child in sorted(base_path.iterdir())
        if child.is_dir()
        and not child.name.startswith(".")
        and has_robot_folders(child, robot_ids)
        and (child / str(robot_ids[0]) / "posterior_pdf_3D_cluster.csv").exists()
    ]


def choose_experiment_folder_gui(base_path: Path, candidates: list[Path]) -> Path | None:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError as exc:
        print(f"GUI folder picker unavailable ({exc}); using terminal prompt.")
        return None

    root = None
    try:
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        selected = filedialog.askdirectory(
            parent=root,
            initialdir=str(base_path),
            title="Select experiment subfolder",
            mustexist=True,
        )
    except Exception as exc:
        print(f"GUI folder picker unavailable ({exc}); using terminal prompt.")
        return None
    finally:
        if root is not None:
            root.destroy()

    if not selected:
        raise SystemExit("No experiment selected.")

    selected_path = Path(selected).expanduser().resolve()
    valid_paths = {candidate.resolve(): candidate for candidate in candidates}
    if selected_path not in valid_paths:
        raise FileNotFoundError(
            f"Selected folder is not a valid experiment subfolder under {base_path}: {selected_path}"
        )
    return valid_paths[selected_path]


def choose_experiment_folder(base_path: Path, candidates: list[Path], use_gui: bool = True) -> Path:
    if use_gui:
        selected = choose_experiment_folder_gui(base_path, candidates)
        if selected is not None:
            return selected

    print(f"Available experiment folders under {base_path}:")
    for idx, candidate in enumerate(candidates):
        print(f"  [{idx}] {candidate.name}")

    while True:
        try:
            choice = input("Choose experiment folder index (q to quit): ").strip()
        except EOFError as exc:
            raise SystemExit(
                "No experiment selected. Pass an experiment folder path or use --experiment-index."
            ) from exc

        if choice.lower() == "q":
            raise SystemExit("No experiment selected.")
        try:
            idx = int(choice)
        except ValueError:
            print("Please enter a numeric index.")
            continue
        if 0 <= idx < len(candidates):
            return candidates[idx]
        print(f"Index out of range. Choose 0 to {len(candidates) - 1}.")


def resolve_experiment_folder(
    selected_path: Path | None,
    robot_ids: tuple[int, ...],
    experiment_index: int | None,
    use_gui: bool = True,
) -> tuple[Path, Path]:
    selected_path = (selected_path or DEFAULT_BASE_PATH).expanduser().resolve()

    if (selected_path / "param_log.csv").exists() and selected_path.name in {
        str(robot_id) for robot_id in robot_ids
    }:
        exp_folder = selected_path.parent
        return exp_folder.parent, exp_folder

    if has_robot_folders(selected_path, robot_ids):
        return selected_path.parent, selected_path

    candidates = experiment_candidates(selected_path, robot_ids)
    if not candidates:
        raise FileNotFoundError(f"No experiment subfolder found in {selected_path}")
    if experiment_index is None:
        return selected_path, choose_experiment_folder(selected_path, candidates, use_gui)
    if experiment_index < 0 or experiment_index >= len(candidates):
        raise IndexError(
            f"Experiment index {experiment_index} out of range; found {len(candidates)} subfolders"
        )
    return selected_path, candidates[experiment_index]


def infer_grid_shape(
    map_size: int,
    params: dict[str, float | list[float]],
    rows_arg: int | None,
    cols_arg: int | None,
) -> tuple[int, int]:
    if rows_arg is not None and cols_arg is not None:
        if rows_arg * cols_arg != map_size:
            raise ValueError(f"--rows * --cols must equal {map_size}")
        return rows_arg, cols_arg

    nbins = params.get("nbins")
    if isinstance(nbins, list) and len(nbins) >= 3:
        # The MATLAB visualizations use the spatial X/Y bins from positions 2 and 3.
        rows = int(nbins[1])
        cols = int(nbins[2])
        if rows * cols == map_size:
            return rows, cols

    side = int(round(np.sqrt(map_size)))
    if side * side == map_size:
        return side, side

    raise ValueError(f"Could not infer grid shape for {map_size} map cells")


def load_experiment(args: argparse.Namespace) -> ExperimentData:
    robot_ids = tuple(args.robot_ids)
    base_path, exp_folder = resolve_experiment_folder(
        args.path,
        robot_ids,
        args.experiment_index,
        args.gui_picker,
    )
    reference_robot = args.reference_robot
    reference_path = exp_folder / str(reference_robot)

    params_path = reference_path / "param_log.csv"
    if not params_path.exists():
        raise FileNotFoundError(f"Missing param_log.csv: {params_path}")

    params = parse_param_log(params_path)
    if "map_nbr" not in params:
        raise KeyError(f"map_nbr not found in {params_path}")

    cluster_raw = load_csv(reference_path / "posterior_pdf_3D_cluster.csv")
    pdf_maps = load_csv(reference_path / "posterior_pdf_3D.csv")
    if cluster_raw.shape[1] < 3:
        raise ValueError(f"Expected cluster data in {reference_path / 'posterior_pdf_3D_cluster.csv'}")
    if pdf_maps.shape[1] == 0:
        raise ValueError(f"Expected belief data in {reference_path / 'posterior_pdf_3D.csv'}")

    cluster_maps = cluster_raw[:, 2:]
    rows, cols = infer_grid_shape(cluster_maps.shape[1], params, args.rows, args.cols)

    trajectories: dict[int, np.ndarray] = {}
    for robot_id in robot_ids:
        traj_path = exp_folder / str(robot_id) / "trajectory.csv"
        trajectory = load_csv(traj_path)
        if trajectory.shape[1] < 3:
            raise ValueError(f"Expected at least 3 columns in {traj_path}")
        trajectories[robot_id] = trajectory

    source_xy = parse_source_position(reference_path / "source_pos.csv", params)
    map_nbr = int(params["map_nbr"])

    return ExperimentData(
        base_path=base_path,
        exp_folder=exp_folder,
        robot_ids=robot_ids,
        reference_robot=reference_robot,
        map_nbr=map_nbr,
        rows=rows,
        cols=cols,
        obstacles=load_obstacles(map_nbr, workspace_root()),
        cluster_maps=cluster_maps,
        pdf_maps=pdf_maps,
        trajectories=trajectories,
        source_xy=source_xy,
    )


def matrix_from_row(row: np.ndarray, rows: int, cols: int) -> np.ndarray:
    return np.reshape(row[: rows * cols], (rows, cols))


def setup_axis(ax: plt.Axes) -> None:
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(*WORLD_X_LIMITS)
    ax.set_ylim(*WORLD_Y_LIMITS)
    ax.set_facecolor(AX_FACE)
    ax.set_xlabel("X (m)", color=TEXT_COLOR)
    ax.set_ylabel("Y (m)", color=TEXT_COLOR)
    ax.set_xticks(np.arange(0, 17, 2))
    ax.set_yticks(np.arange(0, 5, 1))
    ax.grid(True, color=GRID_COLOR, linewidth=0.6, alpha=0.65, zorder=0)
    ax.tick_params(colors="#4c5663", labelsize=9)
    for spine in ax.spines.values():
        spine.set_color(SPINE_COLOR)
        spine.set_linewidth(0.9)


def draw_environment(ax: plt.Axes, data: ExperimentData) -> None:
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

    for x1, y1, x2, y2 in data.obstacles:
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

    source_x, source_y = data.source_xy
    ax.plot(
        source_x,
        source_y,
        marker="*",
        markersize=18,
        markeredgecolor="white",
        markerfacecolor="white",
        linestyle="None",
        zorder=8,
    )
    ax.plot(
        source_x,
        source_y,
        marker="*",
        markersize=14,
        markeredgecolor=SOURCE_COLOR,
        markerfacecolor=SOURCE_COLOR,
        linestyle="None",
        zorder=9,
    )


def draw_iteration(
    fig: plt.Figure,
    axes: tuple[plt.Axes, plt.Axes],
    data: ExperimentData,
    iter_idx: int,
    cbars: list,
) -> None:
    for cbar in cbars:
        cbar.remove()
    cbars.clear()

    ax_cluster, ax_pdf = axes
    ax_cluster.cla()
    ax_pdf.cla()

    iteration = iter_idx + 1
    cluster_mat = matrix_from_row(data.cluster_maps[iter_idx], data.rows, data.cols)
    pdf_mat = matrix_from_row(data.pdf_maps[iter_idx], data.rows, data.cols)

    im_cluster = ax_cluster.imshow(
        cluster_mat,
        extent=[CELL_X_MIN, CELL_X_MAX, CELL_Y_MIN, CELL_Y_MAX],
        origin="lower",
        interpolation="nearest",
        cmap=CLUSTER_CMAP,
        norm=CLUSTER_NORM,
        zorder=1,
    )
    ax_cluster.set_title("Cluster assignment + trajectory", fontsize=13, fontweight="bold", color=TEXT_COLOR)

    for robot_idx, robot_id in enumerate(data.robot_ids):
        trajectory = data.trajectories[robot_id]
        mask = np.abs(trajectory[:, 0] - iteration) < 1e-3
        if not np.any(mask):
            continue
        points = trajectory[mask]
        color = ROBOT_COLORS.get(robot_id, DEFAULT_TRAJ_COLORS[robot_idx % len(DEFAULT_TRAJ_COLORS)])
        ax_cluster.plot(
            points[:, 1],
            points[:, 2],
            "-",
            color=color,
            linewidth=2.3,
            alpha=0.95,
            zorder=7,
        )
        ax_cluster.plot(
            points[:, 1],
            points[:, 2],
            "o",
            color=color,
            markersize=4.5,
            markeredgecolor="white",
            markeredgewidth=0.5,
            alpha=0.95,
            zorder=8,
        )
        ax_cluster.plot(
            points[0, 1],
            points[0, 2],
            "o",
            color=color,
            markersize=8,
            markeredgecolor="black",
            markeredgewidth=1.0,
            zorder=9,
        )
        ax_cluster.plot(
            points[-1, 1],
            points[-1, 2],
            "s",
            color=color,
            markersize=7,
            markeredgecolor="black",
            markeredgewidth=1.0,
            zorder=9,
        )

    draw_environment(ax_cluster, data)
    setup_axis(ax_cluster)

    pdf_vmax = float(np.nanmax(pdf_mat))
    if not np.isfinite(pdf_vmax) or pdf_vmax <= 0:
        pdf_vmax = 1.0
    im_pdf = ax_pdf.imshow(
        pdf_mat,
        extent=[CELL_X_MIN, CELL_X_MAX, CELL_Y_MIN, CELL_Y_MAX],
        origin="lower",
        interpolation="nearest",
        cmap=PDF_CMAP,
        vmin=0.0,
        vmax=pdf_vmax,
        zorder=1,
    )
    ax_pdf.set_title("Posterior belief map", fontsize=13, fontweight="bold", color=TEXT_COLOR)
    draw_environment(ax_pdf, data)
    setup_axis(ax_pdf)

    cbars.append(fig.colorbar(im_cluster, ax=ax_cluster, fraction=0.035, pad=0.02, ticks=[0, 1, 2, 3]))
    cbars[-1].ax.set_yticklabels(["Free", "R101", "R102", "R103"])
    cbars.append(fig.colorbar(im_pdf, ax=ax_pdf, fraction=0.035, pad=0.02))
    cbars[-1].set_label("Belief", color=TEXT_COLOR)

    fig.suptitle(
        f"{data.exp_folder.name}   |   iteration {iteration}/{len(data.cluster_maps)}   |   map {data.map_nbr}",
        fontsize=15,
        fontweight="bold",
        color=TEXT_COLOR,
    )


def run(data: ExperimentData, args: argparse.Namespace) -> None:
    n_iters = min(len(data.cluster_maps), len(data.pdf_maps))
    if args.max_iters is not None:
        n_iters = min(n_iters, args.max_iters)
    if n_iters <= 0:
        raise ValueError("No iterations to display")

    fig, axes_arr = plt.subplots(1, 2, figsize=(14.5, 5.2), facecolor="white", constrained_layout=True)
    axes = (axes_arr[0], axes_arr[1])
    cbars: list = []

    if args.save:
        out_dir = args.output_dir or (data.exp_folder / "clustering_result_frames")
        out_dir.mkdir(parents=True, exist_ok=True)
        for idx in range(n_iters):
            draw_iteration(fig, axes, data, idx, cbars)
            fig.savefig(out_dir / f"{idx + 1:03d}.png", dpi=args.dpi)
            print(f"Saved frame {idx + 1}/{n_iters}")
        print(f"Frames saved to {out_dir}")
        plt.close(fig)
        return

    state = {"quit": False, "step": 1}

    def on_key(event) -> None:
        if event.key == "q":
            state["quit"] = True
        elif event.key in {"left", "backspace"}:
            state["step"] = -1
        else:
            state["step"] = 1
        event.canvas.stop_event_loop()

    def on_close(_event) -> None:
        state["quit"] = True

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", on_close)
    plt.show(block=False)
    print("Press any key for next iteration, left/backspace for previous, q to quit.")

    idx = 0
    while not state["quit"]:
        draw_iteration(fig, axes, data, idx, cbars)
        fig.canvas.draw()
        fig.canvas.flush_events()
        state["step"] = 1
        fig.canvas.start_event_loop(0)

        if state["quit"]:
            break
        next_idx = idx + state["step"]
        if next_idx < 0:
            idx = 0
        elif next_idx >= n_iters:
            break
        else:
            idx = next_idx

    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step through clustering results and posterior belief maps.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=None,
        help="Base folder, experiment folder, or robot folder. If omitted, DEFAULT_BASE_PATH is used.",
    )
    parser.add_argument(
        "--experiment-index",
        type=int,
        default=None,
        help="Subfolder index when path is a base folder. If omitted, a picker window is shown.",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Use the terminal prompt instead of the experiment folder picker window.",
    )
    parser.add_argument("--robot-ids", nargs="+", type=int, default=list(DEFAULT_ROBOT_IDS), help="Robot IDs to load.")
    parser.add_argument("--reference-robot", type=int, default=REFERENCE_ROBOT, help="Robot folder used for map CSVs.")
    parser.add_argument("--rows", type=int, default=None, help="Map grid rows. Inferred if omitted.")
    parser.add_argument("--cols", type=int, default=None, help="Map grid columns. Inferred if omitted.")
    parser.add_argument("--max-iters", type=int, default=None, help="Limit displayed iterations.")
    parser.add_argument("--save", action="store_true", help="Save frames as PNGs instead of opening an interactive viewer.")
    parser.add_argument("--output-dir", type=Path, default=None, help="Frame output directory for --save.")
    parser.add_argument("--dpi", type=int, default=150, help="Saved frame DPI.")
    args = parser.parse_args()
    args.gui_picker = not args.no_gui
    args.robot_ids = tuple(args.robot_ids)
    if args.reference_robot not in args.robot_ids:
        args.robot_ids = (args.reference_robot, *args.robot_ids)
    if args.output_dir is not None:
        args.output_dir = args.output_dir.expanduser().resolve()
    return args


def main() -> None:
    args = parse_args()
    data = load_experiment(args)
    displayed_iters = min(len(data.cluster_maps), len(data.pdf_maps))
    if args.max_iters is not None:
        displayed_iters = min(displayed_iters, args.max_iters)
    print(f"Experiment folder: {data.exp_folder}")
    print(f"Reference robot: {data.reference_robot}")
    print(f"map_nbr from param_log: {data.map_nbr}")
    print(f"Loaded obstacles: {len(data.obstacles)}")
    print(f"Displayed iterations: {displayed_iters}")
    run(data, args)


if __name__ == "__main__":
    main()
