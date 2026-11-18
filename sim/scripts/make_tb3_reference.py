#!/usr/bin/env python3
"""Build the free-space reference of turtlebot3_world from complete exploration runs.

    python3 make_tb3_reference.py <run_dir> [<run_dir> ...]

A 5 cm cell is free in the reference if it is free in at least half of the final maps
(final_map.npz written by monitor.py). Isolated free cells outside the arena are dropped
by keeping only the largest connected region. Output: sim/worlds/tb3_world_free.npz
"""
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
RES = 0.05
X0, Y0, N = -3.0, -3.0, 120   # 6 x 6 m window around the arena

xs = X0 + (np.arange(N) + .5) * RES
X, Y = np.meshgrid(xs, Y0 + (np.arange(N) + .5) * RES)
votes = np.zeros(X.shape, int)
for d in sys.argv[1:]:
    m = np.load(os.path.join(d, 'final_map.npz'))
    g, r, ox, oy = m['data'], float(m['res']), float(m['ox']), float(m['oy'])
    ci = ((X - ox) / r).astype(int)
    ri = ((Y - oy) / r).astype(int)
    ok = (ci >= 0) & (ci < g.shape[1]) & (ri >= 0) & (ri < g.shape[0])
    free = np.zeros(X.shape, bool)
    free[ok] = g[ri[ok], ci[ok]] == 0
    votes += free
free = votes * 2 >= len(sys.argv[1:])
lab, n = ndimage.label(free)
free = lab == (np.argmax(ndimage.sum(free, lab, range(1, n + 1))) + 1)
out = os.path.join(HERE, '..', 'worlds', 'tb3_world_free.npz')
np.savez_compressed(out, bits=np.packbits(free.ravel()), h=N, w=N, x0=X0, y0=Y0, res=RES)
print(f'{len(sys.argv) - 1} maps, free area {free.sum() * RES * RES:.2f} m2 -> {out}')
