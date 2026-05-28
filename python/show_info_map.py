#!/usr/bin/env python3
"""
Interactive viewer for GSL experiment results.
Combines show_info_map.m and show_info_wind_map.m.

Reads param_log.csv; if windInfo == 1 it also shows wind panels.
Press any key to advance to the next belief update, 'q' to quit.
Run with --save to write every frame as a PNG instead of interactive display.

Usage:
    python show_info_map.py <robot_path> [--save]

Example:
    python show_info_map.py ../Results/test/source_in_middle/9/1779438975263/101
"""

import argparse
import os
import sys
import tkinter as tk
from tkinter import filedialog

import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Configuration — edit these to match your setup
# ---------------------------------------------------------------------------
GENERAL_FOLDER = '../Results/test/source_in_middle/9'
ROBOT_SUBDIR   = '101'   # appended after the user selects an experiment folder
# ---------------------------------------------------------------------------

plt.rcParams.update({
    'font.family':       'sans-serif',
    'font.size':         10,
    'axes.titlesize':    11,
    'axes.titleweight':  'semibold',
    'axes.labelsize':    10,
    'figure.facecolor':  'white',
    'axes.facecolor':    'white',
    'savefig.facecolor': 'white',
    'savefig.dpi':       150,
    'savefig.bbox':      'tight',
})

MEAS_CMAP = 'plasma'
# White at 0, deepening through yellow → orange → red → dark red
PDF_CMAP  = mcolors.LinearSegmentedColormap.from_list(
    'belief', ['white', '#ffffb2', '#fecc5c', '#fd8d3c', '#f03b20', '#bd0026'])
WIND_CMAP = 'turbo'
SRC_COLOR = '#27AE60'
OBS_COLOR = 'black'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def parse_param_log(filepath):
    params = {}
    def _read(path):
        with open(path) as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue
                vals = []
                for v in parts[1:]:
                    try:
                        vals.append(float(v))
                    except ValueError:
                        pass
                if vals:
                    params[parts[0]] = vals[0] if len(vals) == 1 else vals
    _read(filepath)
    if 'source_position' not in params:
        bk = os.path.join(os.path.dirname(filepath), 'source_pos.csv')
        if os.path.exists(bk):
            _read(bk)
    return params


def load_csv(path):
    rows = []
    with open(path) as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split(',')]
            try:
                rows.append([float(p) for p in parts if p != ''])
            except ValueError:
                pass
    if not rows:
        return np.empty((0, 1))
    ncols = max(len(r) for r in rows)
    arr = np.full((len(rows), ncols), np.nan)
    for i, r in enumerate(rows):
        arr[i, :len(r)] = r
    return arr


def load_obstacles(map_nbr, workspace_root):
    m = int(map_nbr)
    path = os.path.join(workspace_root, 'openfoam', 'test_maps',
                        f'map{m}', f'map{m}_wind0.75', f'{m}_obstacles.txt')
    obstacles = []
    if not os.path.exists(path):
        return obstacles
    with open(path) as f:
        lines = f.readlines()
    for line in lines[1:]:
        parts = line.strip().split()
        if len(parts) < 4:
            continue
        try:
            obstacles.append(tuple(float(p) for p in parts[:4]))
        except ValueError:
            pass
    return obstacles


def find_iteration_column(meas, n_iters):
    """Return the column index that stores the belief-update iteration number."""
    for c in range(meas.shape[1]):
        col = meas[:, c]
        if (np.all(np.isnan(col) | (col == np.floor(col))) and
                int(np.nanmax(col)) == n_iters - 1 and
                np.nanmin(col) >= 0):
            return c
    # Fallback: the column whose values span [0, n_iters-1] most closely
    return None


# ---------------------------------------------------------------------------
# Axis setup (MATLAB orientation: Y horizontal 0-4, X vertical 2-13 inverted)
# ---------------------------------------------------------------------------

def setup_map_ax(ax, ymin, ymax, xmin, xmax):
    """Horizontal = Y physical (ymin at right), vertical = X physical (xmin at top)."""
    ax.set_xlim(ymax, ymin)   # flip: ymin=0 at right, ymax=4 at left
    ax.set_ylim(xmax, xmin)   # invert: xmin=2 at top, xmax=13 at bottom
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('X (m)')
    ax.set_aspect('equal')


def draw_obstacles(ax, obstacles):
    """Obstacles in plot coords (Y horizontal, X vertical)."""
    for x1, y1, x2, y2 in obstacles:
        # Rectangle corner in data coords: (y1_phys, x1_phys)
        ax.add_patch(mpatches.Rectangle(
            (y1, x1), y2 - y1, x2 - x1,
            facecolor=OBS_COLOR, edgecolor='none', zorder=3,
        ))


def show_image(ax, mat, ymin, ymax, xmin, xmax, cmap, vmin=0, vmax=None):
    """Display mat matching MATLAB imagesc(y,x,mat)+view(180,90)+YDir=reverse.
    Col 0 (Y=ymin) appears at right; row 0 (X=xmin) appears at top."""
    return ax.imshow(
        mat,
        extent=[ymin, ymax, xmax, xmin],  # [left, right, bottom, top]
        origin='upper',
        aspect='auto',
        cmap=cmap,
        vmin=vmin,
        vmax=vmax,
    )


def matlab_reshape(row, nbins_x, nbins_y):
    """Equivalent of MATLAB reshape(row, nbins_x, nbins_y) (column-major)."""
    return np.reshape(row, (nbins_x, nbins_y), order='F')


# ---------------------------------------------------------------------------
# Panel drawing
# ---------------------------------------------------------------------------

def draw_measurements_panel(ax, pos_hist, meas, gas_feature,
                             ymin, ymax, xmin, xmax, obstacles, src):
    """Panel 1: full trajectory + all measurements (drawn once, static). Returns scatter handle."""
    ax.cla()
    col = 2 if gas_feature == 2 else 1     # MATLAB col3 or col2 (1-indexed)
    c_vals = meas[:, col] if meas.shape[1] > col else meas[:, 1]
    sc = ax.scatter(pos_hist[:, 2], pos_hist[:, 1],   # Y→x-axis, X→y-axis
                    s=20, c=c_vals, cmap=MEAS_CMAP, vmin=0,
                    linewidths=0, alpha=0.85, zorder=2)
    ax.plot(src[1], src[0], marker='*', color=SRC_COLOR, markersize=16,
            linewidth=0, markeredgecolor='white', markeredgewidth=0.6, zorder=4)
    draw_obstacles(ax, obstacles)
    setup_map_ax(ax, ymin, ymax, xmin, xmax)
    ax.set_title('Trajectory & Measurements')
    return sc


def draw_wind_panel(ax, U, V, x_wind, y_wind,
                    ymin, ymax, xmin, xmax, obstacles, src,
                    robot_pos, trajectory, ds=5):
    """Panel 2 (wind only): wind speed colour map + quiver direction. Returns image handle."""
    ax.cla()
    Speed = np.sqrt(U**2 + V**2)
    im = show_image(ax, Speed, ymin, ymax, xmin, xmax, WIND_CMAP, vmin=0)

    # Quiver: X_mesh=Y-physical coords (horizontal axis), Y_mesh=X-physical coords
    X_mesh, Y_mesh = np.meshgrid(y_wind, x_wind)
    ax.quiver(X_mesh[::ds, ::ds], Y_mesh[::ds, ::ds],
              V[::ds, ::ds], U[::ds, ::ds],
              color='k', linewidth=0.8, scale=None,
              scale_units='xy', angles='xy', zorder=3)

    draw_obstacles(ax, obstacles)
    ax.plot(src[1], src[0], marker='*', color=SRC_COLOR, markersize=14,
            linewidth=0, markeredgecolor='white', markeredgewidth=0.6, zorder=5)
    ax.plot(robot_pos[2], robot_pos[1], 'o', color='magenta',
            markersize=8, markeredgecolor='k', zorder=5)
    ax.plot(trajectory[:, 2], trajectory[:, 1], '--',
            color='#FF99C8', linewidth=1, zorder=4)
    setup_map_ax(ax, ymin, ymax, xmin, xmax)
    ax.set_title('Wind field (U, V)')
    return im


def draw_wind_uncertainty_panel(ax, W, x_wind, y_wind,
                                ymin, ymax, xmin, xmax, obstacles, src,
                                robot_pos, trajectory):
    """Panel 3 (wind only): wind estimation uncertainty. Returns image handle."""
    ax.cla()
    im = show_image(ax, W, ymin, ymax, xmin, xmax, WIND_CMAP, vmin=0)
    draw_obstacles(ax, obstacles)
    ax.plot(src[1], src[0], marker='*', color=SRC_COLOR, markersize=14,
            linewidth=0, markeredgecolor='white', markeredgewidth=0.6, zorder=5)
    ax.plot(robot_pos[2], robot_pos[1], 'o', color='magenta',
            markersize=8, markeredgecolor='k', zorder=5)
    ax.plot(trajectory[:, 2], trajectory[:, 1], '--',
            color='#FF99C8', linewidth=1, zorder=4)
    setup_map_ax(ax, ymin, ymax, xmin, xmax)
    ax.set_title('Wind estimation uncertainty')
    return im


def draw_pdf_panel(ax, pdf_mat, nbins_x, nbins_y,
                   ymin, ymax, xmin, xmax, obstacles, src,
                   robot_pos, iteration):
    """Panel: posterior PDF as heatmap. Returns image handle."""
    ax.cla()
    mat = matlab_reshape(pdf_mat, nbins_x, nbins_y)
    im = show_image(ax, mat, ymin, ymax, xmin, xmax, PDF_CMAP, vmin=0)
    draw_obstacles(ax, obstacles)
    ax.plot(src[1], src[0], marker='*', color=SRC_COLOR, markersize=16,
            linewidth=0, markeredgecolor='white', markeredgewidth=0.6, zorder=5)
    ax.plot(robot_pos[2], robot_pos[1], 'o', color='magenta',
            markersize=8, markeredgecolor='k', zorder=5)
    setup_map_ax(ax, ymin, ymax, xmin, xmax)
    ax.set_title(f'Posterior PDF — iter {iteration}')
    return im


def draw_belief_panel(ax, pdf_mat, nbins_x, nbins_y,
                      ymin, ymax, xmin, xmax, obstacles, src,
                      robot_pos, trajectory, pos_hist, meas,
                      rel_meas, gas_feature, idx_until, iter_idx,
                      cluster_row=None):
    iteration = iter_idx + 1    # 1-based label
    """Panel: thresholded belief map + trajectory + measurements + clusters."""
    ax.cla()

    # Thresholded PDF
    info = pdf_mat.copy()
    if cluster_row is not None and len(cluster_row) >= 2:
        ratio = cluster_row[1]
        hi, lo = np.nanmax(info), np.nanmin(info)
        thresh = (hi - lo) * ratio + lo
        info[info < thresh] = 0

    mat = matlab_reshape(info, nbins_x, nbins_y)
    im = show_image(ax, mat, ymin, ymax, xmin, xmax, PDF_CMAP, vmin=0)

    # Measurements up to this iteration
    if idx_until > 0:
        if gas_feature == 2 and rel_meas is not None:
            c_vals = rel_meas[iter_idx, :idx_until]
        else:
            col = 2 if gas_feature == 2 else 1
            c_vals = meas[:idx_until, col] if meas.shape[1] > col else meas[:idx_until, 1]
        ax.scatter(pos_hist[:idx_until, 2], pos_hist[:idx_until, 1],
                   s=20, c=c_vals, cmap=MEAS_CMAP, vmin=0,
                   linewidths=0.3, edgecolors='k', alpha=0.9, zorder=4)

    ax.plot(trajectory[:, 2], trajectory[:, 1], '--',
            color='#FF99C8', linewidth=1, zorder=3)
    draw_obstacles(ax, obstacles)
    ax.plot(src[1], src[0], marker='*', color=SRC_COLOR, markersize=16,
            linewidth=0, markeredgecolor='white', markeredgewidth=0.6, zorder=6)
    ax.plot(robot_pos[2], robot_pos[1], 'o', color='magenta',
            markersize=8, markeredgecolor='k', zorder=6)

    # Cluster bounding boxes
    if cluster_row is not None and len(cluster_row) >= 2:
        n_clusters = int(cluster_row[0])
        for k in range(n_clusters):
            base = 2 + k * 4
            if base + 3 < len(cluster_row):
                cx1, cy1, cx2, cy2 = (cluster_row[base], cluster_row[base+1],
                                       cluster_row[base+2], cluster_row[base+3])
                # coords are (x1,y1,x2,y2) physical → rectangle at (y1,x1) in plot
                ax.add_patch(mpatches.Rectangle(
                    (cy1, cx1), cy2 - cy1, cx2 - cx1,
                    fill=False, edgecolor='#FFB300', linewidth=2, zorder=7,
                ))

    setup_map_ax(ax, ymin, ymax, xmin, xmax)
    ax.set_title(f'Belief clustering — iter {iteration}')
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('X (m)')
    return im


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(robot_path, save_frames):
    robot_path = os.path.abspath(robot_path)
    workspace_root = os.path.dirname(  # .../matlab/ → workspace root
        os.path.dirname(os.path.abspath(__file__)))

    # --- Load params ---
    params      = parse_param_log(os.path.join(robot_path, 'param_log.csv'))
    gas_feature = int(params.get('gas_feature', 0))
    map_nbr     = int(params.get('map_nbr', 0))
    wind_info   = int(params.get('windInfo', 0))
    nbins_x     = int(params['nbins'][1])   # spatial X bins
    nbins_y     = int(params['nbins'][2])   # spatial Y bins
    min_maxs    = params['min_maxs']
    xmin, xmax  = min_maxs[2], min_maxs[3]
    ymin, ymax  = min_maxs[4], min_maxs[5]
    src         = params.get('source_position', [0, 0, 0])

    # --- Load data ---
    def _f(name): return os.path.join(robot_path, name)
    def _load(name): return load_csv(_f(name))

    pdf        = _load('posterior_pdf_3D.csv')    # (n_iters, nbins_x*nbins_y)
    meas       = _load('measurement_history.csv')
    pos_hist   = _load('position_history.csv')    # (n_steps, 4): step,X,Y,Z
    positions  = _load('goal_position.csv')       # (n_iters, 5): iter,X,Y,Z,t
    traj_raw   = _load('trajectory.csv')
    trajectory = np.vstack([positions[:1, :traj_raw.shape[1]], traj_raw])

    rel_meas = None
    if gas_feature == 2 and os.path.exists(_f('relative_history_meas.csv')):
        rel_meas = _load('relative_history_meas.csv')

    cluster_boxes = None
    if os.path.exists(_f('clustering_bounding_boxes.csv')):
        cluster_boxes = _load('clustering_bounding_boxes.csv')

    obstacles = load_obstacles(map_nbr, workspace_root) if map_nbr != 0 else []

    # Wind data
    have_wind = False
    if wind_info == 1:
        u_path, v_path, w_path = _f('wind_field_u.csv'), _f('wind_field_v.csv'), _f('wind_field_w.csv')
        if all(os.path.exists(p) for p in (u_path, v_path, w_path)):
            u_all = _load('wind_field_u.csv')
            v_all = _load('wind_field_v.csv')
            w_all = _load('wind_field_w.csv')
            Nx, Ny = 100, 40
            x_wind = np.linspace(xmin, xmax, Nx)
            y_wind = np.linspace(ymin, ymax, Ny)
            have_wind = True
        else:
            print('  windInfo=1 but wind files not found; skipping wind panels.')

    n_iters = len(pdf)

    # Find iteration column in measurement_history
    iter_col = find_iteration_column(meas, n_iters)

    # --- Figure layout ---
    n_panels = 5 if have_wind else 3
    fig, axes = plt.subplots(1, n_panels,
                              figsize=(5.5 * n_panels, 6),
                              constrained_layout=True)
    if n_panels == 1:
        axes = [axes]

    # Panel indices
    if have_wind:
        ax_meas, ax_wind, ax_wunc, ax_pdf, ax_bel = axes
    else:
        ax_meas, ax_pdf, ax_bel = axes

    # Static panel: full trajectory + measurements (colorbar lives for the session)
    sc_meas = draw_measurements_panel(ax_meas, pos_hist, meas, gas_feature,
                                      ymin, ymax, xmin, xmax, obstacles, src)
    fig.colorbar(sc_meas, ax=ax_meas, fraction=0.03, pad=0.02)

    # Colorbars for dynamic panels — tracked so we can remove & recreate each frame
    cbars = {}

    # --- Save folder ---
    if save_frames:
        out_dir = os.path.join(robot_path, 'frames')
        os.makedirs(out_dir, exist_ok=True)

    # --- Key handler for interactive mode ---
    state = {'advance': False, 'quit': False}

    def on_key(event):
        if event.key == 'q':
            state['quit'] = True
        state['advance'] = True
        event.canvas.stop_event_loop()

    if not save_frames:
        fig.canvas.mpl_connect('key_press_event', on_key)
        plt.show(block=False)
        print('Press any key to advance, q to quit.')

    # --- Per-iteration loop ---
    for i in range(n_iters):
        if state['quit']:
            break

        robot_pos = positions[i]          # (iter, X, Y, Z, t)

        # Measurements up to (but not including) this belief update
        idx_until = 0
        if iter_col is not None:
            mask = meas[:, iter_col] <= i - 1
            idx_until = int(np.nansum(mask))

        # Cluster info for this iteration
        cluster_row = None
        if cluster_boxes is not None and i < len(cluster_boxes):
            cluster_row = cluster_boxes[i]

        # Remove colorbars from previous frame before redrawing
        for cb in cbars.values():
            cb.remove()
        cbars.clear()

        # Wind panels
        if have_wind and i < len(u_all):
            U = matlab_reshape(u_all[i], Nx, Ny)
            V = matlab_reshape(v_all[i], Nx, Ny)
            W = matlab_reshape(w_all[i], Nx, Ny)
            im_wind = draw_wind_panel(ax_wind, U, V, x_wind, y_wind,
                                      ymin, ymax, xmin, xmax, obstacles, src,
                                      robot_pos, trajectory)
            cbars['wind'] = fig.colorbar(im_wind, ax=ax_wind, fraction=0.03, pad=0.02)
            im_wunc = draw_wind_uncertainty_panel(ax_wunc, W, x_wind, y_wind,
                                                  ymin, ymax, xmin, xmax, obstacles, src,
                                                  robot_pos, trajectory)
            cbars['wunc'] = fig.colorbar(im_wunc, ax=ax_wunc, fraction=0.03, pad=0.02)

        # Posterior PDF panel
        im_pdf = draw_pdf_panel(ax_pdf, pdf[i], nbins_x, nbins_y,
                                ymin, ymax, xmin, xmax, obstacles, src,
                                robot_pos, i + 1)
        cbars['pdf'] = fig.colorbar(im_pdf, ax=ax_pdf, fraction=0.03, pad=0.02)

        # Belief clustering panel
        im_bel = draw_belief_panel(ax_bel, pdf[i], nbins_x, nbins_y,
                                   ymin, ymax, xmin, xmax, obstacles, src,
                                   robot_pos, trajectory, pos_hist, meas,
                                   rel_meas, gas_feature, idx_until, i,
                                   cluster_row=cluster_row)
        cbars['bel'] = fig.colorbar(im_bel, ax=ax_bel, fraction=0.03, pad=0.02)

        fig.suptitle(f'Iteration {i + 1} / {n_iters}', fontsize=13)

        if save_frames:
            fig.savefig(os.path.join(out_dir, f'{i+1:03d}.png'))
            print(f'  Saved frame {i+1}/{n_iters}')
        else:
            fig.canvas.draw()
            fig.canvas.flush_events()
            fig.canvas.start_event_loop(0)   # wait for keypress
            if state['quit']:
                break
            state['advance'] = False

    if save_frames:
        print(f'Frames saved to {out_dir}/')
    plt.close('all')


def select_experiment_folder(initial_dir):
    """Open a GUI folder-picker (like MATLAB uigetdir) and return the chosen path."""
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    folder = filedialog.askdirectory(
        title='Select experiment folder',
        initialdir=os.path.abspath(initial_dir) if os.path.isdir(initial_dir) else os.getcwd(),
    )
    root.destroy()
    return folder or None


def main():
    parser = argparse.ArgumentParser(
        description='Interactive GSL belief map viewer (combines show_info_map + show_info_wind_map).',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('robot_path', nargs='?', default=None,
                        help='Path to the robot data sub-directory (e.g. .../101). '
                             'If omitted, a folder picker opens.')
    parser.add_argument('--save', action='store_true',
                        help='Save all frames as PNGs instead of interactive display')
    args = parser.parse_args()

    if args.robot_path:
        path = os.path.abspath(args.robot_path)
    else:
        experiment_dir = select_experiment_folder(GENERAL_FOLDER)
        if not experiment_dir:
            sys.exit('No folder selected.')
        path = os.path.join(experiment_dir, ROBOT_SUBDIR)

    if not os.path.isdir(path):
        sys.exit(f'Error: {path} is not a directory')

    run(path, save_frames=args.save)


if __name__ == '__main__':
    main()
