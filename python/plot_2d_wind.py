#!/usr/bin/env python3
"""
Plot a 2-D wind map CSV from the OpenFOAM test maps.

Default input template:
    /home/wjin/Workspace/Multi-robot_GSL/openfoam/test_maps/map%k/map%k_wind0.75/wind_map_2D_%k.csv

Usage:
    python3 plot_2d_wind.py --map 1
    python3 plot_2d_wind.py --map 5 --save
    python3 plot_2d_wind.py --map 9 --mode quiver --downsample 4
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


DEFAULT_MAP = 5
DEFAULT_TEMPLATE = (
    "/home/wjin/Workspace/Multi-robot_GSL/openfoam/test_maps/"
    "map%k/map%k_wind0.75/wind_map_2D_%k.csv"
)
DEFAULT_WIND_SPEED = "0.75"


def path_from_template(template: str, map_idx: int) -> Path:
    return Path(template.replace("%k", str(map_idx))).expanduser()


def workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


def load_wind_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    required = {"X", "Y", "Ux", "Uy"}
    if data.dtype.names is None or not required.issubset(data.dtype.names):
        raise ValueError(f"{path} must contain columns: X,Y,Ux,Uy")

    x = np.unique(data["X"])
    y = np.unique(data["Y"])
    x.sort()
    y.sort()

    u = np.full((len(y), len(x)), np.nan)
    v = np.full_like(u, np.nan)
    x_index = {value: idx for idx, value in enumerate(x)}
    y_index = {value: idx for idx, value in enumerate(y)}

    for row in data:
        yi = y_index[row["Y"]]
        xi = x_index[row["X"]]
        u[yi, xi] = row["Ux"]
        v[yi, xi] = row["Uy"]

    return x, y, u, v


def load_obstacles(map_idx: int, wind_speed: str, root: Path) -> list[tuple[float, float, float, float]]:
    path = (
        root
        / "openfoam"
        / "test_maps"
        / f"map{map_idx}"
        / f"map{map_idx}_wind{wind_speed}"
        / f"{map_idx}_obstacles.txt"
    )
    if not path.exists():
        print(f"Warning: obstacle file not found: {path}")
        return []

    obstacles: list[tuple[float, float, float, float]] = []
    with path.open() as f:
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


def mask_obstacles(
    x: np.ndarray,
    y: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
    obstacles: list[tuple[float, float, float, float]],
) -> tuple[np.ndarray, np.ndarray]:
    u_masked = u.copy()
    v_masked = v.copy()
    xx, yy = np.meshgrid(x, y)

    for xmin, ymin, xmax, ymax in obstacles:
        inside = (xx >= xmin) & (xx <= xmax) & (yy >= ymin) & (yy <= ymax)
        u_masked[inside] = np.nan
        v_masked[inside] = np.nan

    return u_masked, v_masked


def draw_obstacles(ax, obstacles: list[tuple[float, float, float, float]]) -> None:
    import matplotlib.patches as mpatches

    for xmin, ymin, xmax, ymax in obstacles:
        ax.add_patch(
            mpatches.Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                facecolor="#2f343b",
                edgecolor="white",
                linewidth=0.8,
                zorder=4,
            )
        )


def plot_wind_map(
    map_idx: int,
    wind_csv: Path,
    output: Path | None,
    *,
    show: bool,
    mode: str,
    downsample: int,
    draw_obstacle_boxes: bool,
    mask_obstacle_cells: bool,
    wind_speed: str,
) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg", force=True)

    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.titleweight": "semibold",
            "axes.labelsize": 11,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.dpi": 180,
            "savefig.bbox": "tight",
        }
    )

    x, y, u, v = load_wind_csv(wind_csv)
    obstacles = load_obstacles(map_idx, wind_speed, workspace_root()) if draw_obstacle_boxes else []
    u_plot, v_plot = mask_obstacles(x, y, u, v, obstacles) if mask_obstacle_cells else (u, v)
    speed = np.sqrt(u_plot**2 + v_plot**2)

    fig, ax = plt.subplots(figsize=(11, 4.4), constrained_layout=True)
    mesh = ax.pcolormesh(
        x,
        y,
        speed,
        cmap="turbo",
        shading="gouraud",
        vmin=0,
        zorder=1,
    )

    if mode in {"stream", "both"}:
        ax.streamplot(
            x,
            y,
            np.ma.masked_invalid(u_plot),
            np.ma.masked_invalid(v_plot),
            color="#20242a",
            density=1.25,
            linewidth=0.8,
            arrowsize=0.8,
            broken_streamlines=True,
            zorder=3,
        )

    if mode in {"quiver", "both"}:
        step = max(1, int(downsample))
        ax.quiver(
            x[::step],
            y[::step],
            u_plot[::step, ::step],
            v_plot[::step, ::step],
            color="black",
            pivot="mid",
            scale_units="xy",
            scale=None,
            width=0.0025,
            alpha=0.82,
            zorder=3,
        )

    if draw_obstacle_boxes:
        draw_obstacles(ax, obstacles)

    ax.set_title(f"2D Wind Map {map_idx}")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_xlim(float(np.nanmin(x)), float(np.nanmax(x)))
    ax.set_ylim(float(np.nanmin(y)), float(np.nanmax(y)))
    ax.set_xticks([3, 6, 9, 12])
    ax.set_yticks([0, 1, 2, 3, 4])
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="white", alpha=0.28, linewidth=0.6)

    cbar = fig.colorbar(mesh, ax=ax, pad=0.02, aspect=30)
    cbar.set_label("|U| (m/s)")

    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output)
        print(f"Saved wind plot: {output}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot a 2-D OpenFOAM wind CSV.")
    parser.add_argument(
        "-m",
        "--map",
        dest="map_idx",
        type=int,
        default=DEFAULT_MAP,
        help=f"map index replacing %%k in the input path (default: {DEFAULT_MAP})",
    )
    parser.add_argument(
        "--template",
        default=DEFAULT_TEMPLATE,
        help="input path template; every %%k is replaced with --map",
    )
    parser.add_argument(
        "--file",
        type=Path,
        help="exact wind CSV path; overrides --template",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="output image path; providing this also saves the figure",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="save the figure to wind_map_2D_<map>.png beside this script",
    )
    parser.add_argument(
        "--mode",
        choices=("stream", "quiver", "both"),
        default="stream",
        help="wind direction overlay style (default: stream)",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=5,
        help="quiver grid downsample step when --mode is quiver/both (default: 5)",
    )
    parser.add_argument(
        "--wind-speed",
        default=DEFAULT_WIND_SPEED,
        help=f"wind-speed folder suffix used for obstacles (default: {DEFAULT_WIND_SPEED})",
    )
    parser.add_argument(
        "--no-obstacles",
        action="store_true",
        help="do not draw or mask obstacle rectangles",
    )
    parser.add_argument(
        "--no-mask-obstacles",
        action="store_true",
        help="draw obstacles but keep wind values underneath them",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save without displaying; use with --save or --output",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    wind_csv = args.file.expanduser() if args.file else path_from_template(args.template, args.map_idx)
    if not wind_csv.exists():
        raise FileNotFoundError(f"Wind CSV not found: {wind_csv}")

    show = not args.no_show
    save = args.save or args.output is not None
    if not show and not save:
        raise ValueError("--no-show needs --save or --output; otherwise there is nothing to do.")

    output = None
    if save:
        output = (
            args.output.expanduser()
            if args.output is not None
            else Path(__file__).resolve().with_name(f"wind_map_2D_{args.map_idx}.png")
        )

    plot_wind_map(
        args.map_idx,
        wind_csv,
        output,
        show=show,
        mode=args.mode,
        downsample=args.downsample,
        draw_obstacle_boxes=not args.no_obstacles,
        mask_obstacle_cells=not args.no_obstacles and not args.no_mask_obstacles,
        wind_speed=args.wind_speed,
    )


if __name__ == "__main__":
    main()
