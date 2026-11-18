#!/usr/bin/env python3
"""Overview figure of the test worlds with their start poses.

    python3 plot_worlds.py <out.png>
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import world_raster  # noqa: E402

fig = plt.figure(figsize=(13, 6.6), facecolor='#fcfcfb')
gs = fig.add_gridspec(2, 6, height_ratios=[1, 1.25])
axs = [fig.add_subplot(gs[0, 0:2]), fig.add_subplot(gs[0, 2:4]), fig.add_subplot(gs[0, 4:6]),
       fig.add_subplot(gs[1, 0:2]), fig.add_subplot(gs[1, 2:4]), fig.add_subplot(gs[1, 4:6])]
order = ['tb3_world', 'square', 'four_rooms', 'corridor', 'corridor_taper', 'three_rooms']
for ax, w in zip(axs, order):
    R = world_raster.raster(w)
    H, W = R['inside'].shape
    img = np.where(R['inside'], 1.0, 0.25)
    ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower', interpolation='nearest',
              extent=(R['x0'], R['x0'] + W * R['res'], R['y0'], R['y0'] + H * R['res']))
    S = list(csv.DictReader(open(os.path.join(HERE, '..', 'starts', f'{w}.csv'))))[:8]
    x = np.array([float(s['x']) for s in S])
    y = np.array([float(s['y']) for s in S])
    yaw = np.array([float(s['yaw']) for s in S])
    ax.quiver(x, y, np.cos(yaw), np.sin(yaw), color='#2a78d6', scale=28, width=0.006)
    ax.plot(x, y, 'o', color='#2a78d6', ms=4)
    rr, cc = np.nonzero(R['inside'])
    ax.set_xlim(R['x0'] + cc.min() * R['res'] - .3, R['x0'] + (cc.max() + 1) * R['res'] + .3)
    ax.set_ylim(R['y0'] + rr.min() * R['res'] - .3, R['y0'] + (rr.max() + 1) * R['res'] + .3)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(f"{w}\n{R['inside'].sum() * R['res'] ** 2:.0f} m² free", fontsize=10, color='#0b0b0b')
fig.tight_layout()
fig.savefig(sys.argv[1], dpi=110, facecolor=fig.get_facecolor())
