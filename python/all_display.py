#!/usr/bin/env python3
"""
Plot robot trajectory, entropy and concentration measurements for all
experiments in a results folder.  Automatically adapts to single and
multi-robot cases.

Usage:
    python all_display.py <results_dir> [--workspace <workspace_root>] [--show]

Example:
    python all_display.py ../Results/Multi-GSL/REGION_GROWING_2D_BALANCED_center/1
"""

import argparse
import math
import os
import sys

# ---------------------------------------------------------------------------
# Configuration — edit these to run the script without command-line arguments
# ---------------------------------------------------------------------------
DEFAULT_RESULTS_DIR = '/home/wjin/Workspace/Multi-robot_GSL/Results/test/edf_entropy_spread_visit_list_on/9'
SHOW_PLOTS = True   # set True to display figures interactively after saving
# ---------------------------------------------------------------------------

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Visual style
# ---------------------------------------------------------------------------
plt.rcParams.update({
    'font.family':          'sans-serif',
    'font.size':            11,
    'axes.titlesize':       13,
    'axes.titleweight':     'semibold',
    'axes.labelsize':       11,
    'axes.spines.top':      False,
    'axes.spines.right':    False,
    'axes.facecolor':       'white',
    'figure.facecolor':     'white',
    'grid.alpha':           0.35,
    'grid.linewidth':       0.6,
    'grid.color':           '#BBBBBB',
    'legend.framealpha':    0.92,
    'legend.edgecolor':     '#CCCCCC',
    'legend.fontsize':      9,
    'xtick.labelsize':      10,
    'ytick.labelsize':      10,
    'savefig.facecolor':    'white',
    'savefig.dpi':          180,
    'savefig.bbox':         'tight',
})

# Per-robot color cycle (tab10 subset)
ROBOT_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                '#9467bd', '#8c564b', '#e377c2', '#17becf']
MEAS_CMAP  = 'plasma'   # colormap for concentration scatter
OBS_FC     = '#CDD5DF'  # obstacle fill
OBS_EC     = '#8FA3B8'  # obstacle edge
SRC_COLOR  = '#27AE60'  # ground-truth source
MAP_COLOR  = '#E74C3C'  # MAP estimate


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def parse_param_log(filepath):
    """Parse a param_log.csv file (equivalent of MATLAB my_text2struct)."""
    params = {}

    def _read_file(path):
        with open(path) as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue
                key = parts[0]
                values = []
                for v in parts[1:]:
                    try:
                        values.append(float(v))
                    except ValueError:
                        pass
                if values:
                    params[key] = values[0] if len(values) == 1 else values

    _read_file(filepath)

    if 'source_position' not in params:
        backup = os.path.join(os.path.dirname(filepath), 'source_pos.csv')
        if os.path.exists(backup):
            _read_file(backup)

    return params


def load_csv(path):
    """Load a comma-separated numeric file, ignoring malformed rows."""
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(',')
            try:
                rows.append([float(v) for v in parts])
            except ValueError:
                pass
    if not rows:
        return np.empty((0, 1))
    max_cols = max(len(r) for r in rows)
    arr = np.full((len(rows), max_cols), np.nan)
    for i, r in enumerate(rows):
        arr[i, :len(r)] = r
    return arr


def load_obstacles(map_nbr, workspace_root):
    """Return list of (x1, y1, x2, y2) obstacle rectangles."""
    m = int(map_nbr)
    path = os.path.join(
        workspace_root, 'openfoam', 'test_maps',
        f'map{m}', f'map{m}_wind0.75', f'{m}_obstacles.txt',
    )
    obstacles = []
    if not os.path.exists(path):
        print(f'  Warning: obstacle file not found: {path}')
        return obstacles
    with open(path) as f:
        lines = f.readlines()
    for line in lines[1:]:          # skip header line (obstacle count)
        parts = line.strip().split()
        if len(parts) < 4:
            continue
        try:
            obstacles.append(tuple(float(p) for p in parts[:4]))
        except ValueError:
            pass
    return obstacles


def draw_obstacles(ax, obstacles):
    for x1, y1, x2, y2 in obstacles:
        ax.add_patch(mpatches.Rectangle(
            (x1, y1), x2 - x1, y2 - y1,
            facecolor=OBS_FC, edgecolor=OBS_EC, linewidth=0.8, zorder=0,
        ))


def load_robot_data(robot_path):
    """Load all CSV data files for one robot and return as a dict."""
    def _load(name):
        return load_csv(os.path.join(robot_path, name))

    position_history  = _load('position_history.csv')
    measurement_history = _load('measurement_history.csv')
    goal_positions    = _load('goal_position.csv')
    trajectory        = _load('trajectory.csv')
    measurements      = _load('measurements.csv')
    entropy           = _load('entropy.csv')
    expected_ap       = _load('expected_a_posteriori.csv')
    max_ap            = _load('max_a_posteriori.csv')

    # Zero out Z column (col index 3) for positions and trajectory
    if goal_positions.shape[1] > 3:
        goal_positions[:, 3] = 0.0
    if trajectory.shape[1] > 3:
        trajectory[:, 3] = 0.0

    # Prepend first goal position to trajectory (as MATLAB does)
    trajectory = np.vstack([goal_positions[:1, :trajectory.shape[1]], trajectory])

    # Replace NaN in last row of expected_ap with max_ap values
    if expected_ap.shape[0] > 0 and max_ap.shape[0] > 0:
        for col in [1, 2]:
            if expected_ap.shape[1] > col and max_ap.shape[1] > col:
                if np.isnan(expected_ap[-1, col]):
                    expected_ap[-1, col] = max_ap[-1, col]

    return dict(
        position_history=position_history,
        measurement_history=measurement_history,
        goal_positions=goal_positions,
        trajectory=trajectory,
        measurements=measurements,
        entropy=entropy,
        expected_ap=expected_ap,
        max_ap=max_ap,
    )


# ---------------------------------------------------------------------------
# Axis helpers
# ---------------------------------------------------------------------------

def _setup_map_ax(ax):
    ax.set_xlim(2.25, 13.25)
    ax.set_ylim(0, 4)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')


def _meas_column(data, gas_feature):
    mh = data['measurement_history']
    col = 2 if gas_feature == 1 else 1
    return mh[:, col] if mh.shape[1] > col else mh[:, 0]


def _scatter_measurements(ax, data, gas_feature, vmin=None, vmax=None):
    ph = data['position_history']
    sc = ax.scatter(
        ph[:, 1], ph[:, 2],
        s=18, c=_meas_column(data, gas_feature),
        cmap=MEAS_CMAP, vmin=vmin, vmax=vmax,
        alpha=0.85, linewidths=0, zorder=2,
    )
    return sc


def _plot_robot_on_ax(ax, data, params, obstacles, robot_id,
                      gas_feature, robot_color, vmin=None, vmax=None,
                      add_labels=True):
    """Draw all elements for one robot onto ax."""
    sc = _scatter_measurements(ax, data, gas_feature, vmin=vmin, vmax=vmax)

    traj = data['trajectory']
    ax.plot(traj[:, 1], traj[:, 2],
            color=robot_color, linewidth=1.8, alpha=0.85,
            label=f'Robot {robot_id}' if add_labels else None, zorder=3)

    src = params.get('source_position', [0, 0, 0])
    ax.plot(src[0], src[1],
            marker='*', color=SRC_COLOR, markersize=20, linewidth=0,
            markeredgecolor='white', markeredgewidth=0.8,
            label='Source' if add_labels else None, zorder=5)

    max_ap = data['max_ap']
    ax.plot(max_ap[-1, 1], max_ap[-1, 2],
            marker='X', color=MAP_COLOR, markersize=14, linewidth=0,
            markeredgecolor='white', markeredgewidth=0.8,
            label='MAP estimate' if add_labels else None, zorder=5)

    gp = data['goal_positions']
    ax.plot(gp[0, 1], gp[0, 2],
            'o', color=robot_color, markersize=9,
            markerfacecolor='white', markeredgewidth=2,
            label='Start' if add_labels else None, zorder=4)

    draw_obstacles(ax, obstacles)
    return sc


# ---------------------------------------------------------------------------
# Main experiment processor
# ---------------------------------------------------------------------------

def process_experiment(folder_name, workspace_root, show=False):
    # Discover robot sub-directories (numeric names)
    robot_dirs = []
    for entry in os.scandir(folder_name):
        if entry.is_dir():
            try:
                robot_dirs.append((int(entry.name), entry.path))
            except ValueError:
                pass
    if not robot_dirs:
        print(f'  No robot directories in {folder_name}')
        return
    robot_dirs.sort()

    # Load params from first robot to get global settings
    first_params = parse_param_log(os.path.join(robot_dirs[0][1], 'param_log.csv'))
    gas_feature    = int(first_params.get('gas_feature', 0))
    n_robots       = int(first_params.get('number_of_robots', 1))
    is_multi       = n_robots > 1

    # Honour number_of_robots (matches MATLAB loop: for j=1:params.number_of_robots)
    # Extra dirs (e.g. passive nodes) exist but lack data files.
    robot_dirs = robot_dirs[:n_robots]

    # Load all robot data up front (fixes MATLAB scoping bug in f25all loop)
    all_data   = []
    all_params = []
    for robot_id, robot_path in robot_dirs:
        all_data.append(load_robot_data(robot_path))
        all_params.append(parse_param_log(os.path.join(robot_path, 'param_log.csv')))

    obstacles = load_obstacles(
        all_params[0].get('map_nbr', 0), workspace_root
    )

    # Global concentration range (consistent colormap across all robots)
    all_conc = np.concatenate([_meas_column(d, gas_feature) for d in all_data])
    vmin = float(np.nanpercentile(all_conc, 2))
    vmax = float(np.nanpercentile(all_conc, 98))

    def robot_color(i):
        return ROBOT_COLORS[i % len(ROBOT_COLORS)]

    # -----------------------------------------------------------------------
    # Figure 1 — concentration over time + distribution
    # -----------------------------------------------------------------------
    fig_meas, (ax_time, ax_hist) = plt.subplots(
        2, 1, figsize=(9, 6), constrained_layout=True)
    fig_meas.suptitle('Concentration Measurements', fontsize=14)

    for i, ((robot_id, _), data) in enumerate(zip(robot_dirs, all_data)):
        c    = robot_color(i)
        vals = _meas_column(data, gas_feature)
        ax_time.plot(vals, color=c, linewidth=1.2, alpha=0.85,
                     label=f'Robot {robot_id}')
        ax_hist.hist(vals, bins=40, color=c, alpha=0.55, edgecolor='none',
                     label=f'Robot {robot_id}')

    ax_time.set_title('Concentration over time')
    ax_time.set_xlabel('Step')
    ax_time.set_ylabel('Concentration')
    ax_time.legend()

    ax_hist.set_title('Concentration distribution')
    ax_hist.set_xlabel('Concentration')
    ax_hist.set_ylabel('Count')
    ax_hist.legend()

    # -----------------------------------------------------------------------
    # Figure 2 — trajectory + measurements (all robots overlapped)
    # -----------------------------------------------------------------------
    fig_traj, ax_traj = plt.subplots(figsize=(13, 5), constrained_layout=True)
    ax_traj.set_title('Trajectory and Measurements', pad=10)

    sc = None
    for i, ((robot_id, _), data, params) in enumerate(
            zip(robot_dirs, all_data, all_params)):
        sc = _plot_robot_on_ax(
            ax_traj, data, params, obstacles, robot_id,
            gas_feature, robot_color=robot_color(i),
            vmin=vmin, vmax=vmax, add_labels=True,
        )

    _setup_map_ax(ax_traj)
    if sc is not None:
        cbar = fig_traj.colorbar(sc, ax=ax_traj, fraction=0.022, pad=0.01)
        cbar.set_label('Concentration', fontsize=10)
        cbar.ax.tick_params(labelsize=9)
    ax_traj.legend(loc='upper left')

    # -----------------------------------------------------------------------
    # Figure 3 — source estimation evolution (X, Y, entropy)
    # -----------------------------------------------------------------------
    fig_est, axes_est = plt.subplots(
        3, 1, figsize=(6, 9), constrained_layout=True)
    fig_est.suptitle('Source Estimation Evolution', fontsize=14)

    for i, ((robot_id, _), data, params) in enumerate(
            zip(robot_dirs, all_data, all_params)):
        c      = robot_color(i)
        eap    = data['expected_ap']
        map_ap = data['max_ap']
        ent    = data['entropy']
        src    = params.get('source_position', [0, 0, 0])
        min_maxs = params.get('min_maxs', [0] * 10)
        if isinstance(min_maxs, float):
            min_maxs = [min_maxs]

        sfx = f' — Robot {robot_id}' if n_robots > 1 else ''

        n_map = len(map_ap)

        # X estimate — implicit row index as x, ground truth as [0,N] line
        # (mirrors MATLAB: plot(expected_a_posteriori(:,2)) etc.)
        axes_est[0].plot(eap[:, 1],    '-o', color=c, markersize=4,
                         linewidth=1.5, label=f'Expected{sfx}')
        axes_est[0].plot(map_ap[:, 1], '--s', color=c, markersize=4,
                         linewidth=1.5, alpha=0.7, label=f'MAP{sfx}')
        if i == 0:
            axes_est[0].plot([0, n_map], [src[0], src[0]], color=SRC_COLOR,
                             linewidth=1.5, linestyle=':', zorder=0,
                             label='Ground truth')
        if len(min_maxs) >= 4:
            axes_est[0].set_ylim(min_maxs[2], min_maxs[3])

        # Y estimate
        axes_est[1].plot(eap[:, 2],    '-o', color=c, markersize=4,
                         linewidth=1.5)
        axes_est[1].plot(map_ap[:, 2], '--s', color=c, markersize=4,
                         linewidth=1.5, alpha=0.7)
        if i == 0:
            axes_est[1].plot([0, n_map], [src[1], src[1]], color=SRC_COLOR,
                             linewidth=1.5, linestyle=':', zorder=0)
        if len(min_maxs) >= 6:
            axes_est[1].set_ylim(min_maxs[4], min_maxs[5])

        # Entropy
        axes_est[2].plot(ent[:, 0], color=c, linewidth=1.5,
                         label=f'Robot {robot_id}')

    axes_est[0].set_title('X position estimate')
    axes_est[0].set_ylabel('X (m)')
    axes_est[0].grid(True)
    axes_est[0].legend(fontsize=8)

    axes_est[1].set_title('Y position estimate')
    axes_est[1].set_ylabel('Y (m)')
    axes_est[1].grid(True)

    axes_est[2].set_title('Posterior entropy')
    axes_est[2].set_ylabel('Entropy')
    axes_est[2].set_xlabel('Update step')
    axes_est[2].grid(True)
    axes_est[2].legend(fontsize=9)

    # -----------------------------------------------------------------------
    # Figure 4 (multi-robot only) — individual per-robot subplots
    # -----------------------------------------------------------------------
    fig_ind = None
    if is_multi:
        n_cols = math.ceil(math.sqrt(n_robots))
        n_rows = math.ceil(n_robots / n_cols)
        fig_ind, axes_ind = plt.subplots(
            n_rows, n_cols,
            figsize=(6.5 * n_cols, 5 * n_rows),
            constrained_layout=True,
            squeeze=False,
        )
        fig_ind.suptitle('Individual Robot Trajectories', fontsize=14)

        for j, ((robot_id, _), data, params) in enumerate(
                zip(robot_dirs, all_data, all_params)):
            row, col = divmod(j, n_cols)
            ax = axes_ind[row][col]
            ax.set_title(f'Robot {robot_id}')
            sc_j = _plot_robot_on_ax(
                ax, data, params, obstacles, robot_id,
                gas_feature, robot_color=robot_color(j),
                vmin=vmin, vmax=vmax, add_labels=False,
            )
            _setup_map_ax(ax)
            cb = fig_ind.colorbar(sc_j, ax=ax, fraction=0.025, pad=0.01)
            cb.set_label('Conc.', fontsize=9)
            cb.ax.tick_params(labelsize=8)

        for j in range(n_robots, n_rows * n_cols):
            row, col = divmod(j, n_cols)
            axes_ind[row][col].set_visible(False)

    # -----------------------------------------------------------------------
    # Save / show
    # -----------------------------------------------------------------------
    fig_meas.savefig(os.path.join(folder_name, 'measurements_entropy.png'))
    fig_traj.savefig(os.path.join(folder_name, 'trajectory.png'))
    fig_est.savefig(os.path.join(folder_name, 'estimation_evolution.png'))
    if fig_ind is not None:
        fig_ind.savefig(os.path.join(folder_name, 'trajectory_all_robots.png'))

    if show:
        figs = [fig_meas, fig_traj, fig_est]
        if fig_ind is not None:
            figs.append(fig_ind)
        # Tile windows so they don't overlap: two columns, two rows
        positions = [(0, 0), (960, 0), (0, 600), (960, 600)]
        backend = plt.get_backend()
        for fig, (px, py) in zip(figs, positions):
            try:
                mgr = fig.canvas.manager
                if 'Qt' in backend:
                    mgr.window.move(px, py)
                elif 'Tk' in backend:
                    mgr.window.wm_geometry(f'+{px}+{py}')
            except Exception:
                pass
            fig.canvas.draw()
        plt.pause(2)
        plt.close('all')

    saved = ['measurements_entropy.png', 'trajectory.png', 'estimation_evolution.png']
    if fig_ind is not None:
        saved.append('trajectory_all_robots.png')
    print(f'  Saved: {", ".join(saved)}')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Plot GSL experiment results (single or multi-robot).',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('results_dir',
                        nargs='?',
                        default=DEFAULT_RESULTS_DIR,
                        help='Directory containing experiment timestamp folders '
                             f'(default: {DEFAULT_RESULTS_DIR})')
    parser.add_argument('--workspace',
                        default=None,
                        help='Workspace root for obstacle file lookup '
                             '(default: two levels up from this script)')
    parser.add_argument('--show', action='store_true', default=SHOW_PLOTS,
                        help='Display figures interactively (in addition to saving)')
    args = parser.parse_args()

    results_dir = os.path.abspath(args.results_dir)
    if not os.path.isdir(results_dir):
        sys.exit(f'Error: {results_dir} is not a directory')

    if args.workspace:
        workspace_root = os.path.abspath(args.workspace)
    else:
        # Script lives in <workspace>/matlab/ — go one level up
        workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Collect experiment subdirectories (skip hidden and 'datasets')
    exp_folders = sorted(
        e.path for e in os.scandir(results_dir)
        if e.is_dir() and not e.name.startswith('.') and e.name != 'datasets'
    )

    print(f'Found {len(exp_folders)} experiment(s) in {results_dir}')
    for folder in exp_folders:
        print(f'Processing: {os.path.basename(folder)}')
        try:
            process_experiment(folder, workspace_root, show=args.show)
        except Exception as exc:
            import traceback
            print(f'  ERROR: {exc}')
            traceback.print_exc()


if __name__ == '__main__':
    main()
