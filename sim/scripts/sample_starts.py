#!/usr/bin/env python3
"""Sample random start poses: uniform over the free space reachable by the robot, at least CLEAR
from any obstacle and SEP from each other, random yaw.

    python3 sample_starts.py <world> <N> <seed> > sim/starts/<world>.csv
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_raster  # noqa: E402

CLEAR = 0.40
SEP = 0.40

world, N, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
rng = np.random.default_rng(seed)
R = world_raster.raster(world)
rr, cc = np.nonzero(R['main'] & (R['dist'] >= CLEAR))
cand = np.c_[R['x0'] + (cc + .5) * R['res'], R['y0'] + (rr + .5) * R['res']]
pts = []
for k in rng.permutation(len(cand)):
    p = cand[k] + rng.uniform(-R['res'] / 2, R['res'] / 2, 2)
    if all(np.hypot(*(p - q)) >= SEP for q in pts):
        pts.append(p)
    if len(pts) == N:
        break
print('i,x,y,yaw')
for i, (p, a) in enumerate(zip(pts, rng.uniform(-np.pi, np.pi, len(pts)))):
    print(f'{i},{p[0]:.3f},{p[1]:.3f},{a:.3f}')
