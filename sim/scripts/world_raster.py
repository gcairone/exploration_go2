#!/usr/bin/env python3
"""Ground-truth rasters of the test worlds (5 cm grid) and traversability check.

    python3 world_raster.py <world>      # prints free area and connectivity for the robot

raster(world) returns a dict with
  inside  free cells inside the outer walls
  dist    distance [m] of every cell from the nearest obstacle / wall
  main    largest region reachable by the robot centre (dist >= INFL)
  x0, y0, res  grid origin and resolution (world frame)
For the generated worlds the geometry comes from sim/worlds/gen_worlds.py.
turtlebot3_world uses meshes, so its free space is a reference mask
(sim/worlds/tb3_world_free.npz, built by make_tb3_reference.py from complete maps).
"""
import math
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS_DIR = os.path.join(HERE, '..', 'worlds')
sys.path.insert(0, WORLDS_DIR)
import gen_worlds  # noqa: E402

RES = 0.05
INFL = 0.27       # Nav2 lethal footprint (0.22 m) + margin


def _shapes_raster(shapes):
    def extent(sh):
        if sh[0] == 'cyl':
            return sh[1] - sh[3], sh[1] + sh[3], sh[2] - sh[3], sh[2] + sh[3]
        cx, cy, sx, sy, _ = sh
        r = math.hypot(sx, sy) / 2
        return cx - r, cx + r, cy - r, cy + r
    E = np.array([extent(sh) for sh in shapes])
    x0, x1 = E[:, 0].min() - .3, E[:, 1].max() + .3
    y0, y1 = E[:, 2].min() - .3, E[:, 3].max() + .3
    xs = np.arange(x0, x1, RES) + RES / 2
    ys = np.arange(y0, y1, RES) + RES / 2
    X, Y = np.meshgrid(xs, ys)
    occ = np.zeros(X.shape, bool)
    for sh in shapes:
        if sh[0] == 'cyl':
            occ |= np.hypot(X - sh[1], Y - sh[2]) <= sh[3]
        else:
            cx, cy, sx, sy, a = sh
            u = (X - cx) * np.cos(a) + (Y - cy) * np.sin(a)
            v = -(X - cx) * np.sin(a) + (Y - cy) * np.cos(a)
            occ |= (np.abs(u) <= sx / 2) & (np.abs(v) <= sy / 2)
    lab, _ = ndimage.label(~occ)
    inside = (lab > 0) & (lab != lab[0, 0])
    return inside, x0, y0


def raster(world):
    if world == 'tb3_world':
        z = np.load(os.path.join(WORLDS_DIR, 'tb3_world_free.npz'))
        inside = np.unpackbits(z['bits'])[:int(z['h']) * int(z['w'])].reshape(int(z['h']), int(z['w'])).astype(bool)
        x0, y0 = float(z['x0']), float(z['y0'])
    else:
        inside, x0, y0 = _shapes_raster(gen_worlds.WORLDS[world])
    dist = ndimage.distance_transform_edt(inside) * RES
    nav = inside & (dist >= INFL)
    lab, n = ndimage.label(nav)
    sizes = ndimage.sum(nav, lab, range(1, n + 1))
    main = lab == (np.argmax(sizes) + 1) if n else nav
    return dict(inside=inside, dist=dist, main=main, x0=x0, y0=y0, res=RES, n_components=n)


def interior_points(R, margin=0.15):
    """World coordinates of interior cells at least `margin` from walls (used for coverage)."""
    m = R['inside'] & (R['dist'] >= margin)
    rr, cc = np.nonzero(m)
    return R['x0'] + (cc + .5) * R['res'], R['y0'] + (rr + .5) * R['res']


def coverage(grid, res, ox, oy, px, py):
    """Fraction of points (px, py) that are known (free or occupied) in an occupancy grid."""
    ci = ((px - ox) / res).astype(int)
    ri = ((py - oy) / res).astype(int)
    ok = (ci >= 0) & (ci < grid.shape[1]) & (ri >= 0) & (ri < grid.shape[0])
    known = np.zeros(len(px), bool)
    known[ok] = grid[ri[ok], ci[ok]] >= 0
    return known.mean() if len(px) else 0.0


if __name__ == '__main__':
    w = sys.argv[1]
    R = raster(w)
    print(f"{w}: free area {R['inside'].sum() * RES * RES:.2f} m2, "
          f"robot-reachable components (clearance {INFL} m): {R['n_components']}, "
          f"main component {R['main'].sum() * RES * RES:.2f} m2")
