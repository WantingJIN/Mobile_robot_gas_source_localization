#!/usr/bin/env python3
"""
Plot per-iteration belief maps and region allocation for multi-robot GSL experiment.

Layout per iteration (2 rows, GridSpec 2×6):
  Row 0 (each 2/6 cols): [R1 Indiv. Belief] [R2 Indiv. Belief] [R3 Indiv. Belief]
  Row 1 (each 3/6 cols): [Merged Belief   ] [Region Allocation]

Both rows span identical total width. Top plots are each 2/3 the width of bottom plots.

Files used (all relative to exp_folder/<robot_id>/):
  posterior_pdf_pre_merge.csv   : rows=FSM cycles, cols=400 cell probabilities (per robot)
  posterior_pdf_3D.csv          : rows=FSM cycles, cols=400 merged belief intensities (R101 only)
  posterior_pdf_3D_cluster.csv  : rows=FSM cycles, cols=2 goal_xy + 400 cluster IDs (R101 only)
  goal_position.csv             : cols=[cycle, goal_x, goal_y, ...]
  measurement_history.csv       : cols=[step, concentration, norm_conc, FSM_iteration]
  position_history.csv          : cols=[step, x, y, heading] — rows align 1:1 with above
  source_pos.csv                : "source_position x y z"
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import os

# ── Configuration ────────────────────────────────────────────────────────────
EXP_FOLDER = (
    '/home/wjin/Workspace/Multi-robot_GSL/Results/DARS_2026/'
    'proposed_method_generate_plot/9/1777571374870'
)
ROBOT_IDS   = [101, 102, 103]
ROBOT_NAMES = ['Robot 1', 'Robot 2', 'Robot 3']

# Grid setup (matching MATLAB script)
N_COLS  = 20
N_ROWS  = 20
X_EDGES = np.linspace(3, 12.5, N_COLS + 1)
Y_EDGES = np.linspace(0.16, 3.8,  N_ROWS + 1)

X_LIM = (3.0, 13.0)
Y_LIM = (0.0,  4.0)

# ── Color scheme ─────────────────────────────────────────────────────────────
# Full robot colors (used for measurement markers and belief map high end)
ROBOT_COLORS = ['#1a6faf', '#d95f02', '#1b7837']   # blue, orange, green

# Per-robot colormaps: white (low belief / low concentration) → robot color (high)
# Used for BOTH the belief heatmap background AND the measurement scatter dots.
ROBOT_CMAPS = [
    LinearSegmentedColormap.from_list('', ['#ffffff', c], N=256)
    for c in ROBOT_COLORS
]

# Merged belief: white → dark charcoal
MERGED_CMAP = LinearSegmentedColormap.from_list('', ['#ffffff', '#1e2d40'], N=256)

# Cluster region fill colors (lighter than robot colors for region background)
CLUSTER_BASE = {
    0: np.array([0.92, 0.92, 0.92]),
    1: np.array([0.40, 0.66, 0.84]),
    2: np.array([0.99, 0.55, 0.23]),
    3: np.array([0.40, 0.76, 0.44]),
}

DARK_GRAY          = '#3a3a3a'
MIN_VISIBLE_INTENSITY = 0.15


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


# ── Data loading ─────────────────────────────────────────────────────────────
def load_data():
    pre_merge = []
    for rid in ROBOT_IDS:
        path = os.path.join(EXP_FOLDER, str(rid), 'posterior_pdf_pre_merge.csv')
        pre_merge.append(np.loadtxt(path, delimiter=','))

    merged = np.loadtxt(
        os.path.join(EXP_FOLDER, '101', 'posterior_pdf_3D.csv'), delimiter=',')

    cluster_raw  = np.loadtxt(
        os.path.join(EXP_FOLDER, '101', 'posterior_pdf_3D_cluster.csv'), delimiter=',')
    cluster_maps = cluster_raw[:, 2:].astype(int)

    goal_positions = []
    for rid in ROBOT_IDS:
        path = os.path.join(EXP_FOLDER, str(rid), 'goal_position.csv')
        g = np.loadtxt(path, delimiter=',')
        goal_positions.append(g[:, 1:3])

    # Measurement history: [step, concentration, norm_conc, FSM_iteration]
    # Position history:    [step, x, y, heading]   — same row count, 1:1 alignment
    meas_history = []
    pos_history  = []
    for rid in ROBOT_IDS:
        meas_history.append(np.loadtxt(
            os.path.join(EXP_FOLDER, str(rid), 'measurement_history.csv'), delimiter=','))
        pos_history.append(np.loadtxt(
            os.path.join(EXP_FOLDER, str(rid), 'position_history.csv'), delimiter=','))

    # Per-robot global max of raw concentration (col 1) for consistent normalisation
    meas_global_max = [m[:, 1].max() for m in meas_history]

    with open(os.path.join(EXP_FOLDER, '101', 'source_pos.csv')) as f:
        tokens = f.read().split()
    source_xy = (float(tokens[1]), float(tokens[2]))

    return (pre_merge, merged, cluster_maps, goal_positions,
            meas_history, pos_history, meas_global_max, source_xy)


# ── Shared helpers ────────────────────────────────────────────────────────────
def _normalize(arr):
    mn, mx = arr.min(), arr.max()
    return (arr - mn) / (mx - mn) if mx > mn else np.zeros_like(arr)


def _style_ax(ax, source_xy):
    """Obstacles, source marker, axis limits, y-ticks."""
    for ox, oy, ow, oh in [(8.0, 0.0, 0.5, 2.0), (4.5, 2.0, 0.5, 2.0)]:
        ax.add_patch(mpatches.Rectangle(
            (ox, oy), ow, oh, facecolor=DARK_GRAY, edgecolor='none', zorder=3))

    ax.plot(*source_xy, '*', markersize=9,
            markeredgecolor='k', markerfacecolor='k',
            markeredgewidth=0.5, zorder=8)

    ax.set_xlim(*X_LIM)
    ax.set_ylim(*Y_LIM)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', fontsize=13)
    ax.set_yticks([0, 4])
    ax.set_yticklabels(['0 m', '4 m'], fontsize=12)
    ax.tick_params(axis='x', labelsize=12)


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
    Only non-zero readings are drawn.
    """
    mask    = meas[:, 3] <= iteration_limit      # rows up to this iteration
    conc    = meas[mask, 1]                       # raw concentration
    xy      = pos[mask, 1:3]                      # x, y positions

    keep    = conc > 1e-6                         # discard zero / near-zero readings
    if not np.any(keep):
        return

    c_norm = conc[keep] / (global_max + 1e-10)   # normalise 0→1 by per-robot peak

    ax.scatter(xy[keep, 0], xy[keep, 1],
               c=c_norm, cmap=ROBOT_CMAPS[r_idx],
               vmin=0, vmax=1,
               s=18, zorder=5,
               edgecolors='#333333', linewidths=0.25,
               alpha=0.85)


# ── Per-panel plotting ────────────────────────────────────────────────────────
def plot_individual_belief(ax, belief_1d, r_idx, source_xy,
                           meas, pos, global_max, iteration_limit):
    """Belief heatmap (white→robot colour) with measurement scatter overlay."""
    norm = _normalize(belief_1d).reshape(N_ROWS, N_COLS)
    mesh = ax.pcolormesh(X_EDGES, Y_EDGES, norm,
                         cmap=ROBOT_CMAPS[r_idx], vmin=0, vmax=1,
                         zorder=1, rasterized=True)
    _style_ax(ax, source_xy)
    _plot_measurements(ax, meas, pos, global_max, iteration_limit, r_idx)
    sensor_handle = Line2D([0], [0], marker='o', color='w',
                           markerfacecolor=ROBOT_COLORS[r_idx],
                           markeredgecolor='#333333', markeredgewidth=0.3,
                           markersize=5, label='Sensor output')
    ax.legend(handles=[sensor_handle], loc='upper left', fontsize=11,
              framealpha=0.85, handlelength=1.0, borderpad=0.4)
    _add_colorbar(ax, mesh)
    ax.set_title(f'{ROBOT_NAMES[r_idx]} — Individual Belief',
                 fontsize=13, fontweight='bold', pad=4)


def plot_merged_belief(ax, intensity_1d, source_xy):
    """Merged belief heatmap (white → dark charcoal), no measurement overlay."""
    norm = _normalize(intensity_1d).reshape(N_ROWS, N_COLS)
    mesh = ax.pcolormesh(X_EDGES, Y_EDGES, norm,
                         cmap=MERGED_CMAP, vmin=0, vmax=1,
                         zorder=1, rasterized=True)
    _style_ax(ax, source_xy)
    _add_colorbar(ax, mesh)
    ax.set_title('Merged Belief Map', fontsize=13, fontweight='bold', pad=4)


def plot_cluster_allocation(ax, cluster_1d, intensity_1d, goals, source_xy):
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
    _style_ax(ax, source_xy)

    for r_idx, gxy in enumerate(goals):
        if gxy is not None:
            ax.plot(*gxy, 'D', markersize=7, linewidth=0,
                    color=ROBOT_COLORS[r_idx], markeredgecolor='k',
                    markeredgewidth=0.6, zorder=7)

    source_handle = ax.plot([], [], '*', markersize=8,
                            markeredgecolor='k', markerfacecolor='k',
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


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    (pre_merge, merged, cluster_maps, goal_positions,
     meas_history, pos_history, meas_global_max, source_xy) = load_data()

    n_iter     = cluster_maps.shape[0]
    output_dir = os.path.join(EXP_FOLDER, 'output_plots')
    os.makedirs(output_dir, exist_ok=True)
    print(f'Saving {n_iter} plots to: {output_dir}')

    for it in range(n_iter):
        # FSM cycle is 1-indexed; cluster maps are 0-indexed.
        # Show all measurements where measurement_history col3 <= it+1.
        iteration_limit = it + 1

        fig = plt.figure(figsize=(15, 6))
        gs  = GridSpec(2, 6, figure=fig, hspace=0.45, wspace=0.35)

        ax_r1  = fig.add_subplot(gs[0, 0:2])
        ax_r2  = fig.add_subplot(gs[0, 2:4])
        ax_r3  = fig.add_subplot(gs[0, 4:6])
        ax_mrg = fig.add_subplot(gs[1, 0:3])
        ax_cls = fig.add_subplot(gs[1, 3:6])


        # Next goals shown only on cluster panel (row it+2 = one cycle ahead)
        goals = []
        for r_idx in range(3):
            gps = goal_positions[r_idx]
            row = it + 2
            goals.append((gps[row, 0], gps[row, 1]) if row < len(gps) else None)

        # Row 0: individual belief + measurements
        for r_idx, ax in enumerate([ax_r1, ax_r2, ax_r3]):
            plot_individual_belief(
                ax, pre_merge[r_idx][it], r_idx, source_xy,
                meas_history[r_idx], pos_history[r_idx],
                meas_global_max[r_idx], iteration_limit,
            )

        # Row 1
        plot_merged_belief(ax_mrg, merged[it], source_xy)
        plot_cluster_allocation(ax_cls, cluster_maps[it], merged[it], goals, source_xy)

        out_path = os.path.join(output_dir, f'iteration_{it + 1:03d}.pdf')
        plt.savefig(out_path, bbox_inches='tight',
                    facecolor='white', edgecolor='none')
        plt.close(fig)
        print(f'  [{it + 1:2d}/{n_iter}] {os.path.basename(out_path)}')

    print('Done!')


if __name__ == '__main__':
    main()
