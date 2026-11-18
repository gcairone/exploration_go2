#!/usr/bin/env python3
"""Animated GIF of an exploration run recorded with --frames.

    python3 make_gif.py <run_dir> <out.gif> [--title TEXT] [--step 1] [--ms 90] [--width 560]

Shows the slam_toolbox map (unknown grey, free white, occupied black), the ground-truth walls
(thin outline), the robot and its trajectory, frontier cells, the DBSCAN candidates and the
current Nav2 plan. One GIF frame every --step recorded frames (frames are 2 s of simulated time apart).
"""
import argparse
import io
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_raster  # noqa: E402

ROBOT = '#2a78d6'
FRONTIER = '#eb6834'
CANDIDATE = '#1baf7a'
PLAN = '#6250d6'

p = argparse.ArgumentParser()
p.add_argument('run')
p.add_argument('out')
p.add_argument('--title', default='')
p.add_argument('--step', type=int, default=1)
p.add_argument('--ms', type=int, default=90)
p.add_argument('--width', type=int, default=560)
a = p.parse_args()

info = dict(kv.split('=') for kv in open(os.path.join(a.run, 'run.txt')).readline().split())
Z = np.load(os.path.join(a.run, 'frames.npz'))
T, P, meta, maps = Z['t'], Z['pose'], Z['meta'], Z['maps']
R = world_raster.raster(info['world'])
px, py = world_raster.interior_points(R)
H, W = R['inside'].shape
ext_w = (R['x0'], R['x0'] + W * R['res'], R['y0'], R['y0'] + H * R['res'])
rr, cc = np.nonzero(R['inside'])
pad = 0.4
xl = (R['x0'] + cc.min() * R['res'] - pad, R['x0'] + (cc.max() + 1) * R['res'] + pad)
yl = (R['y0'] + rr.min() * R['res'] - pad, R['y0'] + (rr.max() + 1) * R['res'] + pad)
aspect = (yl[1] - yl[0]) / (xl[1] - xl[0])
fig_w = 6.0
HEAD, FOOT = 0.5, 0.4
fig_h = fig_w * aspect + HEAD + FOOT
fig = plt.figure(figsize=(fig_w, fig_h), facecolor='#fcfcfb')
ax = fig.add_axes((0, FOOT / fig_h, 1, fig_w * aspect / fig_h))
handles = [Line2D([], [], color=ROBOT, marker='o', ms=7, lw=1.4, label='robot / path'),
           Line2D([], [], color=FRONTIER, marker='o', ms=3, lw=0, label='frontier cells'),
           Line2D([], [], color=CANDIDATE, marker='o', ms=7, lw=0, label='DBSCAN candidates'),
           Line2D([], [], color=PLAN, lw=1.4, ls=(0, (3, 2)), label='Nav2 plan')]
fig.legend(handles=handles, loc='lower center', ncol=4, frameon=False, fontsize=8, labelcolor='#52514e',
           bbox_to_anchor=(0.5, 0.0), handletextpad=0.4, columnspacing=1.2)

offs = np.r_[0, np.cumsum(meta[:, 3].astype(int) * meta[:, 4].astype(int))]
images = []
idx = list(range(0, len(T), a.step))
if idx[-1] != len(T) - 1:
    idx.append(len(T) - 1)
for i in idx:
    res, ox, oy, h, w = meta[i]
    h, w = int(h), int(w)
    g = maps[offs[i]:offs[i + 1]].reshape(h, w)
    cov = world_raster.coverage(g, res, ox, oy, px, py)
    img = np.full((h, w, 3), 0.80)
    img[g == 0] = 1.0
    img[g > 50] = 0.12
    ax.clear()
    ax.set_facecolor('#cccccc')
    ax.imshow(img, origin='lower', extent=(ox, ox + w * res, oy, oy + h * res), interpolation='nearest')
    ax.contour(np.linspace(ext_w[0], ext_w[1], W), np.linspace(ext_w[2], ext_w[3], H),
               R['inside'].astype(float), [0.5], colors='#52514e', linewidths=0.5, alpha=0.6)
    f, c, pl = Z[f'frontiers_{i}'], Z[f'cands_{i}'], Z[f'plan_{i}']
    if len(f):
        ax.scatter(f[:, 0], f[:, 1], s=4, color=FRONTIER, lw=0)
    if len(pl):
        ax.plot(pl[:, 0], pl[:, 1], color=PLAN, lw=1.2, ls=(0, (3, 2)))
    if len(c):
        ax.scatter(c[:, 0], c[:, 1], s=36, color=CANDIDATE, edgecolors='#fcfcfb', linewidths=1.2, zorder=4)
    tr = P[:i + 1]
    ax.plot(tr[:, 0], tr[:, 1], color=ROBOT, lw=1.4, alpha=0.8)
    x, y, yaw = P[i]
    if np.isfinite(x):
        ax.add_patch(plt.Circle((x, y), 0.12, color=ROBOT, zorder=5))
        ax.plot([x, x + 0.28 * np.cos(yaw)], [y, y + 0.28 * np.sin(yaw)], color=ROBOT, lw=2, zorder=5)
    ax.set_xlim(*xl)
    ax.set_ylim(*yl)
    ax.set_aspect('equal')
    ax.axis('off')
    for t in list(fig.texts):
        t.remove()
    fig.text(0.02, 1 - 0.25 / fig_h, a.title or info['world'], fontsize=11, color='#0b0b0b',
             va='center', ha='left')
    fig.text(0.98, 1 - 0.25 / fig_h, f't = {T[i]:4.0f} s   explored {100 * cov:3.0f} %',
             fontsize=10, color='#52514e', va='center', ha='right', family='monospace')
    buf = io.BytesIO()
    fig.savefig(buf, dpi=a.width / fig_w, facecolor=fig.get_facecolor())
    images.append(Image.open(buf).convert('RGB'))

sample = images[::max(1, len(images) // 8)]
montage = Image.new('RGB', (images[0].width * len(sample), images[0].height))
for k, im in enumerate(sample):
    montage.paste(im, (k * im.width, 0))
pal = montage.quantize(colors=128, method=0)
frames = [im.quantize(palette=pal, dither=0) for im in images]
durations = [a.ms] * (len(frames) - 1) + [2500]
frames[0].save(a.out, save_all=True, append_images=frames[1:], duration=durations, loop=0, optimize=True)
print(f'{a.out}: {len(frames)} frames, {os.path.getsize(a.out) / 1e6:.2f} MB')
