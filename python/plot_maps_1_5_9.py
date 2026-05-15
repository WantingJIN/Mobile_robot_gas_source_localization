"""
Plot environmental maps for maps 1, 5, 9 with obstacles, wind streamlines,
and wind-speed magnitude (Morandi palette).
Reproduces the key visualisation from openfoam_process_for_webots_test_maps.m.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as mpatches
import scipy.io

# ── Morandi palette ──────────────────────────────────────────────────────────
# Muted, desaturated academic tones inspired by Giorgio Morandi.
#   cream-white → dusty rose-tan → muted sage → dusty teal → deep slate-blue
MORANDI_CMAP = mcolors.LinearSegmentedColormap.from_list(
    'morandi',
    [(0.00, '#F0EBE3'),   # warm cream  — calm / near-zero speed
     (0.25, '#C8B8A8'),   # dusty tan
     (0.50, '#98AFAA'),   # muted sage-teal
     (0.75, '#6888A0'),   # dusty blue
     (1.00, '#3D5A78')],  # deep slate-blue — peak speed
)
MORANDI_CMAP.set_bad(color='#6B6360')   # NaN (obstacle interior) → warm dark grey

# Obstacle face / edge / streamline colours kept in the same family
OBS_FACE   = '#4A4540'   # warm charcoal
OBS_EDGE   = '#D0C8C0'   # soft light grey border
STREAM_COL = '#EDE8E2'   # near-cream — visible on the dark end, subtle on the light

# ── Grid definitions (must match the MATLAB script) ────────────────────────
x  = np.round(np.arange(0.2124, 15.9138 + 1e-6, 0.1586), 4)   # 100 pts
y  = np.round(np.arange(0.01,   3.9916  + 1e-6, 0.0632), 4)   #  64 pts
z  = np.round(np.arange(0.0,    2.0     + 1e-6, 0.1),   4)    #  21 pts
Wx = np.round(np.arange(2.75,  12.7418  + 1e-6, 0.1586), 4)   #  64 pts
Wy = y.copy()

Z01_IDX = np.argmin(np.abs(z - 0.1))
WX_IDX  = np.array([np.where(x == w)[0][0] for w in Wx])

BASE  = '../openfoam/test_maps/'
MAPS  = [1, 5, 9]
X_LIM = (3.0, 13.0)


# ── Helpers ──────────────────────────────────────────────────────────────────

def load_obstacle_boxes(folder, map_idx):
    """Return list of (xmin, ymin, xmax, ymax) bounding boxes."""
    txt = os.path.join(folder, f'{map_idx}_obstacles.txt')
    boxes = []
    with open(txt) as fh:
        n = int(fh.readline())
        for _ in range(n):
            boxes.append(tuple(map(float, fh.readline().split())))
    return boxes


def load_obstacle_mask(folder, map_idx):
    """Return (x_1d, y_1d, is_obstacle bool[ny,nx]) — True where solid.

    .mat convention: 0=obstacle, 1=free (OpenFOAM cell presence) → invert.
    Fallback reconstructs bounding boxes.  Both return True = obstacle.
    """
    mat = os.path.join(folder, f'{map_idx}_obstacle.mat')
    if os.path.exists(mat):
        d = scipy.io.loadmat(mat)
        return d['X2'][0, :], d['Y2'][:, 0], ~d['obstacle_map'].astype(bool)
    # Reconstruct from txt
    obs = np.zeros((len(y), len(x)), dtype=bool)
    for (xmin, ymin, xmax, ymax) in load_obstacle_boxes(folder, map_idx):
        obs[np.ix_((y >= ymin) & (y <= ymax),
                   (x >= xmin) & (x <= xmax))] = True
    return x, y, obs


def load_wind_2d(folder, map_idx):
    """Return (wx_1d, wy_1d, u[ny,nx], v[ny,nx]) on the wind grid."""
    mat = os.path.join(folder, 'wind_map.mat')
    if os.path.exists(mat):
        d = scipy.io.loadmat(mat)
        u, v = d['wind_x2D'], d['wind_y2D']
        if 'WX' in d and d['WX'].ndim == 2 and d['WX'].shape == (len(Wy), len(Wx)):
            return d['WX'][0, :], d['WY'][:, 0], u, v
        return Wx, Wy, u, v
    # Reconstruct from 3-D txt dump
    txt = os.path.join(folder, f'{map_idx}.txt')
    print(f'  map {map_idx}: reconstructing wind from {os.path.basename(txt)} …')
    raw   = np.loadtxt(txt, skiprows=1)
    wx3d  = raw[:, 0].reshape(len(y), len(x), len(z), order='F')
    wy3d  = raw[:, 1].reshape(len(y), len(x), len(z), order='F')
    return Wx, Wy, wx3d[:, WX_IDX, Z01_IDX], wy3d[:, WX_IDX, Z01_IDX]


# ── Pre-compute global speed range for a shared colorbar ─────────────────────
all_speeds = []
data_cache = {}
for m in MAPS:
    folder = os.path.join(BASE, f'map{m}', f'map{m}_wind0.75')
    x1d, y1d, obs  = load_obstacle_mask(folder, m)
    wx1d, wy1d, u, v = load_wind_2d(folder, m)
    boxes = load_obstacle_boxes(folder, m)
    data_cache[m] = (x1d, y1d, obs, wx1d, wy1d, u, v, boxes)

    wxmask = (wx1d >= X_LIM[0]) & (wx1d <= X_LIM[1])
    speed  = np.sqrt(u[:, wxmask]**2 + v[:, wxmask]**2)
    # mask obstacle cells on the wind grid
    xmask_obs = (x1d >= X_LIM[0]) & (x1d <= X_LIM[1])
    obs_on_wx  = obs[:, xmask_obs]
    if obs_on_wx.shape[1] == speed.shape[1]:
        speed[obs_on_wx] = np.nan
    all_speeds.append(np.nanmax(speed))

VMAX = max(all_speeds)   # shared upper bound [m/s]


# ── Global style — muted, no background ──────────────────────────────────────
plt.rcParams.update({
    'font.size':          14,
    'axes.spines.top':    False,
    'axes.spines.right':  False,
    'axes.edgecolor':     'k',
    'xtick.color':        'k',
    'ytick.color':        'k',
    'axes.labelcolor':    'k',
    'text.color':         '#403830',
    'figure.facecolor':   'none',
    'axes.facecolor':     'none',
})

# ── Figure — 1 row × 3 cols, single shared colorbar ──────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(18, 4), constrained_layout=True)

pcm_last = None

for i, (ax, map_idx) in enumerate(zip(axes, MAPS)):
    x1d, y1d, obs, wx1d, wy1d, u, v, boxes = data_cache[map_idx]

    # ── 1. Wind magnitude (background) ──────────────────────────────────
    wxmask    = (wx1d >= X_LIM[0]) & (wx1d <= X_LIM[1])
    wx_sub    = wx1d[wxmask]
    u_sub     = u[:, wxmask].copy()
    v_sub     = v[:, wxmask].copy()

    xmask_obs = (x1d >= X_LIM[0]) & (x1d <= X_LIM[1])
    obs_sub   = obs[:, xmask_obs]

    if obs_sub.shape[1] == u_sub.shape[1]:
        u_sub[obs_sub] = np.nan
        v_sub[obs_sub] = np.nan

    speed = np.sqrt(u_sub**2 + v_sub**2)

    pcm_last = ax.pcolormesh(
        wx_sub, wy1d, speed,
        cmap=MORANDI_CMAP,
        vmin=0, vmax=VMAX,
        shading='gouraud',
        zorder=1,
    )

    # ── 2. Obstacles as filled rectangles ───────────────────────────────
    for (xmin, ymin, xmax, ymax) in boxes:
        rx = max(xmin, X_LIM[0])
        rw = min(xmax, X_LIM[1]) - rx
        if rw <= 0:
            continue
        ax.add_patch(mpatches.Rectangle(
            (rx, ymin), rw, ymax - ymin,
            facecolor=OBS_FACE, edgecolor=OBS_EDGE, linewidth=0.8,
            zorder=2,
        ))

    # ── 3. Wind streamlines — black ──────────────────────────────────────
    ax.streamplot(
        wx_sub, wy1d, u_sub, v_sub,
        density=0.7,
        linewidth=0.9,
        color='#555555',
        arrowsize=0.8,
        broken_streamlines=True,
        zorder=3,
    )

    # ── Per-axes formatting ──────────────────────────────────────────────
    for spine in ax.spines.values():
        spine.set_edgecolor('k')

    ax.set_xlim(X_LIM)
    ax.set_ylim(y1d[0], y1d[-1])
    ax.set_xticks([3, 6, 9, 12])
    ax.set_yticks([0, 2, 4])
    ax.tick_params(axis='both', labelsize=16)
    ax.set_xlabel('x [m]', fontsize=17)
    ax.set_title(f'Map {i + 1}', fontsize=17, fontweight='bold',
                 color='#403830', pad=6)
    ax.set_aspect('equal')

    if i == 0:
        ax.set_ylabel('y [m]', fontsize=17)
    else:
        ax.set_yticklabels([])

# ── Single shared colorbar ────────────────────────────────────────────────────
cb = fig.colorbar(pcm_last, ax=axes.tolist(), shrink=0.85, pad=0.02, aspect=25)
cb.set_label('|U| [m/s]', fontsize=17, color='#504840')
n_ticks = 4
cb.set_ticks(np.linspace(0, VMAX, n_ticks))
cb.ax.tick_params(labelsize=16, colors='k')
cb.outline.set_edgecolor('k')

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps_1_5_9.pdf')
plt.savefig(out, bbox_inches='tight', transparent=True)
print(f'Saved → {out}')
plt.show()
