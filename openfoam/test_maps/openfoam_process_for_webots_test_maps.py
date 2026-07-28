#!/usr/bin/env python3
"""
Python version of:
  - openfoam_process_for_webots_test_maps.m
  - openfoam_steadyState_CC_U.m

It reads OpenFOAM cell centers and UMean, generates obstacle, flow definition,
SDF, 2D/3D wind maps, CSV files, .mat files, and diagnostic figures.
"""

from __future__ import annotations

import argparse
import os
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.io import savemat
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist


# =============================================================================
# Helper functions
# =============================================================================


def matlab_range(start: float, step: float, stop: float, decimals: int = 4) -> np.ndarray:
    """Replicate MATLAB start:step:stop, then round."""
    n = int(np.floor((stop - start) / step + 1e-12)) + 1
    return np.round(start + step * np.arange(n), decimals)


def closest_idx(array: np.ndarray, value: float) -> int:
    """Python equivalent of MATLAB closest_idx helper."""
    return int(np.argmin(np.abs(array - value)))


def _internal_field_lines(path: Path) -> tuple[list[str], str | None]:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    start = next((i for i, line in enumerate(lines) if line.strip().startswith("internalField")), None)
    if start is None:
        raise ValueError(f"Missing internalField in {path}")

    internal_line = lines[start].strip()
    uniform_match = re.search(r"uniform\s+(.+?)\s*;", internal_line)
    if uniform_match:
        return [uniform_match.group(1)], "uniform"

    count_idx = None
    count = None
    for i in range(start + 1, len(lines)):
        stripped = lines[i].strip()
        if stripped.isdigit():
            count_idx = i
            count = int(stripped)
            break
    if count_idx is None or count is None:
        raise ValueError(f"Could not find nonuniform internalField size in {path}")

    values: list[str] = []
    for line in lines[count_idx + 1:]:
        stripped = line.strip()
        if stripped in {"", "("}:
            continue
        if stripped in {")", ");"}:
            break
        values.append(stripped)
        if len(values) == count:
            break

    if len(values) != count:
        raise ValueError(f"Expected {count} internalField values in {path}, found {len(values)}")
    return values, None


def read_scalar_file(path: Path, header_lines: int = 22) -> np.ndarray:
    """Read the scalar internalField from an OpenFOAM volScalarField."""
    del header_lines
    values, _ = _internal_field_lines(path)
    return np.asarray([float(v) for v in values], dtype=float)


def read_vector_file(path: Path, header_lines: int = 22) -> np.ndarray:
    """Read the vector internalField from an OpenFOAM volVectorField."""
    del header_lines
    values, _ = _internal_field_lines(path)
    pattern = re.compile(
        r"\(\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*\)"
    )
    vectors: list[tuple[float, float, float]] = []
    for value in values:
        m = pattern.search(value)
        if not m:
            raise ValueError(f"Could not parse vector value {value!r} in {path}")
        vectors.append(tuple(float(v) for v in m.groups()))
    return np.asarray(vectors, dtype=float)


def openfoam_steady_state_cc_u(case_path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """
    Python equivalent of openfoam_steadyState_CC_U(case_path).

    MATLAB behavior:
      - ignores '.', '..', 'constant', 'system', 'VTK'
      - reads U from case_path/10/UMean with HeaderLines=22
      - reads Cx, Cy, Cz from the first remaining time directory with HeaderLines=22
    """
    case_path = Path(case_path)

    excluded = {".", "..", "constant", "system", "VTK"}
    time_dirs = sorted(
        p for p in case_path.iterdir()
        if p.is_dir() and p.name not in excluded
    )
    if not time_dirs:
        raise FileNotFoundError(f"No OpenFOAM time directories found in {case_path}")

    u_file = case_path / "10" / "UMean"
    if not u_file.exists():
        raise FileNotFoundError(f"Missing UMean file: {u_file}")

    cc_dir = time_dirs[0]
    cx_file = cc_dir / "Cx"
    cy_file = cc_dir / "Cy"
    cz_file = cc_dir / "Cz"

    for path in (cx_file, cy_file, cz_file):
        if not path.exists():
            raise FileNotFoundError(f"Missing cell-center file: {path}")

    u = read_vector_file(u_file, header_lines=22)
    ccx = read_scalar_file(cx_file, header_lines=22)
    ccy = read_scalar_file(cy_file, header_lines=22)
    ccz = read_scalar_file(cz_file, header_lines=22)

    cc = np.column_stack((ccx, ccy, ccz))

    if len(cc) != len(u):
        print(
            f"Warning: CC has {len(cc)} rows but U has {len(u)} rows. "
            "Check OpenFOAM files/time folders."
        )

    return cc, u


def parse_obstacle_points(topo_set_dict: Path) -> list[np.ndarray]:
    """Extract obstacle box points from system/topoSetDict."""
    topo_str = topo_set_dict.read_text(encoding="utf-8", errors="ignore")

    # MATLAB used: regexp(str,'box*[^\n\r]+);','match')
    # This version is more tolerant and captures box definitions that span lines.
    box_blocks = re.findall(r"box\s*\([^;]*?\)\s*;", topo_str, flags=re.DOTALL)
    if not box_blocks:
        box_blocks = re.findall(r"box.*?\);", topo_str, flags=re.DOTALL)

    if not box_blocks:
        return []

    point_pattern = re.compile(
        r"\(\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*\)"
    )

    points: list[np.ndarray] = []
    for block in box_blocks:
        for match in point_pattern.finditer(block):
            points.append(np.asarray([float(v) for v in match.groups()], dtype=float))

    return points


def parse_inlet_patch_segments(block_mesh_dict: Path) -> list[dict[str, float | str]]:
    """Read inlet patch x/y extents from system/blockMeshDict."""
    text = block_mesh_dict.read_text(encoding="utf-8", errors="ignore")
    vertices_match = re.search(r"vertices\s*\((.*?)\)\s*;\s*blocks", text, flags=re.DOTALL)
    if not vertices_match:
        return []

    point_pattern = re.compile(
        r"\(\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
        r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*\)"
    )
    vertices = [tuple(float(v) for v in m.groups()) for m in point_pattern.finditer(vertices_match.group(1))]
    if not vertices:
        return []

    segments: list[dict[str, float | str]] = []
    inlet_blocks = re.finditer(
        r"^\s*(inlet\w*)\s*\{(.*?)^\s*\}",
        text,
        flags=re.DOTALL | re.MULTILINE,
    )
    for patch_match in inlet_blocks:
        name = patch_match.group(1)
        body = patch_match.group(2)
        faces_match = re.search(r"faces\s*\((.*?)\)\s*;", body, flags=re.DOTALL)
        if not faces_match:
            continue

        for face_match in re.finditer(r"\(([^()]+)\)", faces_match.group(1)):
            indices = [int(idx) for idx in face_match.group(1).split()]
            if not indices or any(idx >= len(vertices) for idx in indices):
                continue

            xs = [vertices[idx][0] for idx in indices]
            ys = [vertices[idx][1] for idx in indices]
            segments.append({
                "name": name,
                "xmin": min(xs),
                "xmax": max(xs),
                "ymin": min(ys),
                "ymax": max(ys),
            })

    return segments


def apply_inlet_segments_to_flow_def(flow_def: np.ndarray, wx: np.ndarray, wy: np.ndarray,
                                     block_mesh_dict: Path) -> bool:
    """Project map13+ partial inlet patches onto the 2D Webots flow grid."""
    segments = parse_inlet_patch_segments(block_mesh_dict)
    if not segments:
        return False

    # A single "inlet" patch is the legacy map layout. Named partial inlet
    # patches are the map13+ layout and need explicit projection.
    if len(segments) == 1 and segments[0]["name"] == "inlet":
        return False

    x_tol = float(np.diff(wx).mean() / 2) if len(wx) > 1 else 1e-6
    y_tol = float(np.diff(wy).mean() / 2) if len(wy) > 1 else 1e-6

    flow_def[:, 0] = 4
    flow_def[:, -1] = 4
    flow_def[0, :] = 2
    flow_def[-1, :] = 2

    for segment in segments:
        xmin = float(segment["xmin"])
        xmax = float(segment["xmax"])
        ymin = float(segment["ymin"])
        ymax = float(segment["ymax"])
        vertical = abs(xmax - xmin) <= x_tol
        horizontal = abs(ymax - ymin) <= y_tol

        if vertical:
            col = 0 if xmax <= wx.min() + x_tol else -1
            rows = np.where((wy >= ymin - y_tol) & (wy <= ymax + y_tol))[0]
            flow_def[rows, col] = 3
        elif horizontal:
            row = 0 if ymax <= wy.min() + y_tol else -1
            cols = np.where((wx >= xmin - x_tol) & (wx <= xmax + x_tol))[0]
            flow_def[row, cols] = 3

    return True


def nearest_grid_lookup(values: np.ndarray, decimals: int = 4) -> dict[float, int]:
    return {float(np.round(v, decimals)): i for i, v in enumerate(values)}


def save_heatmaps(folder: Path, maps: int, wx: np.ndarray, wy: np.ndarray,
                  wind_x2d: np.ndarray, wind_y2d: np.ndarray) -> None:
    speed = np.hypot(wind_x2d, wind_y2d)

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), facecolor="w")
    for ax, data, title in zip(axes, [wind_x2d, wind_y2d, speed], ["U_x", "U_y", "|U|"]):
        im = ax.imshow(
            data,
            extent=[wx.min(), wx.max(), wy.min(), wy.max()],
            origin="lower",
            aspect="equal",
        )
        ax.set_title(title)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        fig.colorbar(im, ax=ax)
    fig.tight_layout()
    fig.savefig(folder / f"wind_map_2D_{maps}_heatmaps.png", dpi=200)
    plt.close(fig)


def save_quiver(folder: Path, maps: int, wx_grid: np.ndarray, wy_grid: np.ndarray,
                wind_x2d: np.ndarray, wind_y2d: np.ndarray) -> None:
    ds = max(1, round(min(wx_grid.shape) / 40))
    fig, ax = plt.subplots(facecolor="w")
    ax.quiver(
        wx_grid[::ds, ::ds],
        wy_grid[::ds, ::ds],
        wind_x2d[::ds, ::ds],
        wind_y2d[::ds, ::ds],
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Wind vectors (quiver)")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(folder / f"wind_map_2D_{maps}_quiver.png", dpi=200)
    plt.close(fig)


def process_map(general_folder: Path, map_id: int, wind_label: str = "wind0.75") -> None:
    step = 0.1

    # Universal grid
    x = matlab_range(0.2124, 0.1586, 15.9138, 4)
    y = matlab_range(0.01, 0.0632, 3.9916, 4)
    z = matlab_range(0.0, 0.1, 2.0, 4)
    x_grid, y_grid, z_grid = np.meshgrid(x, y, z, indexing="xy")

    folder = general_folder / f"map{map_id}" / f"map{map_id}_{wind_label}"
    folder.mkdir(parents=True, exist_ok=True)

    cc, u = openfoam_steady_state_cc_u(folder)

    # Obstacles
    pts = parse_obstacle_points(folder / "system" / "topoSetDict")
    nbr_obstacles = len(pts) // 2

    with (folder / f"{map_id}_obstacles.txt").open("w", encoding="utf-8") as f:
        f.write(f"{nbr_obstacles}\n")
        for i in range(nbr_obstacles):
            start_pt = pts[2 * i]
            end_pt = pts[2 * i + 1]
            f.write(f"{start_pt[0]:f} {start_pt[1]:f} {end_pt[0]:f} {end_pt[1]:f}\n")

    # Flow definition and SDF grid
    wx = matlab_range(2.75, 0.1586, 12.7418, 4)
    wy = matlab_range(0.01, 0.0632, 3.9916, 4)
    wx_grid, wy_grid = np.meshgrid(wx, wy, indexing="xy")

    flow_def = np.ones(wx_grid.shape, dtype=float)

    for i in range(nbr_obstacles):
        start_pt = pts[2 * i]
        end_pt = pts[2 * i + 1]
        xmin, ymin = start_pt[:2]
        xmax, ymax = end_pt[:2]

        mxmin = closest_idx(wx, xmin)
        mxmax = closest_idx(wx, xmax)
        mymin = closest_idx(wy, ymin)
        mymax = closest_idx(wy, ymax)

        if mxmin > mxmax:
            mxmin, mxmax = mxmax, mxmin
        if mymin > mymax:
            mymin, mymax = mymax, mymin

        flow_def[mymin:mymax + 1, mxmin:mxmax + 1] = 0
        flow_def[mymin:mymax + 1, mxmin] = 5
        flow_def[mymin:mymax + 1, mxmax] = 5
        flow_def[mymin, mxmin:mxmax + 1] = 5
        flow_def[mymax, mxmin:mxmax + 1] = 5

    # Input, output, and walls. Maps 13+ use partial inlet patches that differ
    # from the older single-inlet layout.
    custom_inlets = apply_inlet_segments_to_flow_def(
        flow_def,
        wx,
        wy,
        folder / "system" / "blockMeshDict",
    )
    if not custom_inlets:
        flow_def[:, -1] = 3
        flow_def[:, 0] = 4
        flow_def[0, :] = 2
        flow_def[-1, :] = 2

    # Remove borders between touching obstacles
    for r, c in zip(*np.where(flow_def == 5)):
        if 0 < c < flow_def.shape[1] - 1:
            if flow_def[r, c - 1] == 0 and flow_def[r, c + 1] == 0:
                flow_def[r, c] = 0
        if 0 < r < flow_def.shape[0] - 1:
            if flow_def[r - 1, c] == 0 and flow_def[r + 1, c] == 0:
                flow_def[r, c] = 0

    border_mask = (flow_def == 5) | (flow_def == 2)
    border_points = np.column_stack((wx_grid[border_mask], wy_grid[border_mask]))
    grid_points = np.column_stack((wx_grid.ravel(), wy_grid.ravel()))

    if len(border_points) > 0:
        sdf = cdist(grid_points, border_points).min(axis=1).reshape(wx_grid.shape)
    else:
        sdf = np.ones(wx_grid.shape, dtype=float) * 9000
    sdf[flow_def == 0] *= -1

    savemat(folder / "SDF.mat", {"X": x_grid, "Y": y_grid, "sdf": sdf})
    savemat(folder / "flow_def.mat", {"X": x_grid, "Y": y_grid, "flow_def": flow_def})

    # 2D wind map
    wind_x2d = np.zeros(wx_grid.shape, dtype=float)
    wind_y2d = np.zeros(wx_grid.shape, dtype=float)

    cc_round = np.round(cc, 4)
    x_set = set(float(v) for v in x)
    mask = np.array([(float(row[0]) in x_set) and np.isclose(row[2], 0.1) for row in cc_round])

    wx_lookup = nearest_grid_lookup(wx)
    wy_lookup = nearest_grid_lookup(wy)

    for velocity, center in zip(u[mask], cc_round[mask]):
        ix = wx_lookup.get(float(center[0]))
        iy = wy_lookup.get(float(center[1]))
        if ix is not None and iy is not None:
            wind_x2d[iy, ix] = velocity[0]
            wind_y2d[iy, ix] = velocity[1]

    # Original MATLAB checks flow_def == -50, although -50 is commented out.
    # Keeping the same behavior plus zeroing walls.
    wind_x2d[(flow_def == -50) | (flow_def == 0) | (flow_def == 2) | (flow_def == 5)] = 0
    wind_y2d[(flow_def == -50) | (flow_def == 0) | (flow_def == 2) | (flow_def == 5)] = 0

    savemat(
        folder / "wind_map.mat",
        {"WX": wx_grid, "WY": wy_grid, "wind_x2D": wind_x2d, "wind_y2D": wind_y2d},
    )

    # Match MATLAB's WX(:), WY(:), wind_x2D(:), wind_y2D(:) output order.
    # MATLAB flattens matrices in column-major order, so y varies first for each x.
    df = pd.DataFrame({
        "X": wx_grid.ravel(order="F"),
        "Y": wy_grid.ravel(order="F"),
        "Ux": wind_x2d.ravel(order="F"),
        "Uy": wind_y2d.ravel(order="F"),
    })
    csv_file = folder / f"wind_map_2D_{map_id}.csv"
    df.to_csv(csv_file, index=False)

    save_heatmaps(folder, map_id, wx, wy, wind_x2d, wind_y2d)
    save_quiver(folder, map_id, wx_grid, wy_grid, wind_x2d, wind_y2d)

    # Obstacle map from cell center membership, equivalent to ismembertol(...,'ByRows',true)
    xyz_points = np.column_stack((x_grid.ravel(), y_grid.ravel(), z_grid.ravel()))
    tree = cKDTree(np.round(cc, 4))
    distances, _ = tree.query(np.round(xyz_points, 4), k=1)
    obstacle_flag = distances <= 1e-8

    x2 = x_grid[:, :, 0]
    y2 = y_grid[:, :, 0]
    obstacle_map = (obstacle_flag.astype(float).reshape(len(y), len(x), len(z)) * 2).sum(axis=2) > 20

    # Walls around the map
    obstacle_map[
        np.isclose(x2, step / 2)
        | np.isclose(np.round(x2, 4), 16 - step / 2)
        | np.isclose(y2, step / 2)
        | np.isclose(np.round(y2, 4), 4 - step / 2)
    ] = False

    savemat(folder / f"{map_id}_obstacle.mat", {"X2": x2, "Y2": y2, "obstacle_map": obstacle_map})

    # 3D wind maps
    wind_x = np.zeros((len(y), len(x), len(z)), dtype=float)
    wind_y = np.zeros_like(wind_x)
    wind_z = np.zeros_like(wind_x)

    x_lookup = nearest_grid_lookup(x)
    y_lookup = nearest_grid_lookup(y)
    z_lookup = nearest_grid_lookup(z)

    for velocity, center in zip(u, cc_round):
        ix = x_lookup.get(float(center[0]))
        iy = y_lookup.get(float(center[1]))
        iz = z_lookup.get(float(center[2]))
        if ix is not None and iy is not None and iz is not None:
            wind_x[iy, ix, iz] = velocity[0]
            wind_y[iy, ix, iz] = velocity[1]
            wind_z[iy, ix, iz] = velocity[2]

    # Save data as original text format
    # MATLAB writes wind_x(:), wind_y(:), wind_z(:) in column-major order.
    d = np.vstack((
        wind_x.ravel(order="F"),
        wind_y.ravel(order="F"),
        wind_z.ravel(order="F"),
    ))
    with (folder / f"{map_id}.txt").open("w", encoding="utf-8") as f:
        f.write(f"{d.shape[1]}\n")
        for col in d.T:
            f.write(f"{col[0]:f} {col[1]:f} {col[2]:f}\n")

    # Wind magnitude figure
    mag = np.sqrt(wind_x ** 2 + wind_y ** 2)
    z_idx = min(9, mag.shape[2] - 1)  # MATLAB mag(:,:,10), converted to zero-based index 9
    mag_map = obstacle_map * mag[:, :, z_idx]

    fig, ax = plt.subplots(facecolor="w")
    im = ax.imshow(
        mag_map,
        extent=[x.min(), x.max(), y.min(), y.max()],
        origin="lower",
        aspect="equal",
    )
    fig.colorbar(im, ax=ax)
    ax.set_title("Wind magnitude")
    fig.tight_layout()
    fig.savefig(folder / f"{map_id}_wind.png", dpi=200)
    plt.close(fig)

    # Diagnostic visualization
    fig, axes = plt.subplots(3, 3, figsize=(10, 8), facecolor="w")
    for ax in axes.ravel():
        ax.axis("off")

    diagnostics = [
        (flow_def, "flow def"),
        (sdf, "SDF"),
        (wind_x2d, "wind x"),
        (wind_y2d, "wind y"),
    ]
    positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
    for (data, title), (rr, cc_) in zip(diagnostics, positions):
        axes[rr, cc_].axis("on")
        axes[rr, cc_].imshow(data, origin="lower", aspect="auto")
        axes[rr, cc_].set_title(title)

    fig.tight_layout()
    fig.savefig(folder / f"{map_id}.png", dpi=200)
    plt.close(fig)

    print(f"Finished map {map_id}: {folder}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Process OpenFOAM test maps for Webots.")
    parser.add_argument("--general-folder", default="", help="Base folder containing map folders.")
    parser.add_argument("--maps", type=int, nargs="+", default=[15], help="Map IDs to process, e.g. --maps 0 1 2 16")
    parser.add_argument("--wind-label", default="wind0.75", help="Folder suffix, e.g. wind0.75")
    args = parser.parse_args()

    general_folder = Path(args.general_folder)
    for map_id in args.maps:
        process_map(general_folder, map_id, args.wind_label)


if __name__ == "__main__":
    main()
