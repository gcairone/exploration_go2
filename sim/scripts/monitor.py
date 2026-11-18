#!/usr/bin/env python3
"""Exploration monitor: logs coverage over (simulated) time and decides when a run is over.

    python3 monitor.py --out DIR --world NAME [--stall 150] [--stop 0.99] [--frames]

Every simulated second it appends to DIR/metrics.csv:
  sim_s, wall_s, known_m2 (known cells of /map), coverage (fraction of the ground-truth interior
  that is known, see world_raster.py), x, y (robot in map frame), dist_m (travelled distance).
It writes DIR/done.txt ("<reason> <sim_s>") when coverage >= --stop or when the known area has
not grown by more than 0.2 m2 for --stall seconds. On exit it saves DIR/final_map.npz and,
with --frames, DIR/frames.npz (map, pose, frontiers, candidates and Nav2 plan every 2 s) for make_gif.py.
"""
import argparse
import csv
import math
import os
import struct
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_raster  # noqa: E402


class Monitor(Node):
    def __init__(self, a):
        super().__init__('exploration_monitor',
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        R = world_raster.raster(a.world)
        self.px, self.py = world_raster.interior_points(R)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map = None
        self.frontiers = np.zeros((0, 2), np.float32)
        self.cands = np.zeros((0, 2), np.float32)
        self.plan = np.zeros((0, 2), np.float32)
        self.dist, self.last_xy = 0.0, None
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PointCloud2, '/frontiers', self.on_frontiers, 10)
        self.create_subscription(MarkerArray, '/frontier_candidates', self.on_cands, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.f = open(os.path.join(a.out, 'metrics.csv'), 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['sim_s', 'wall_s', 'known_m2', 'coverage', 'x', 'y', 'dist_m'])
        self.t0w, self.t0s = time.time(), None
        self.best, self.last_gain, self.done = 0.0, None, False
        self.frames, self.last_frame = [], -1e9
        self.create_timer(1.0, self.tick)

    def on_map(self, m):
        self.map = m

    def on_odom(self, o):
        p = o.pose.pose.position
        if self.last_xy:
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def on_frontiers(self, m):
        n = m.width * m.height
        pts = struct.unpack_from(f'<{3 * n}f', bytes(m.data)) if n else ()
        self.frontiers = np.array(pts, np.float32).reshape(-1, 3)[:, :2]

    def on_cands(self, m):
        self.cands = np.array([(k.pose.position.x, k.pose.position.y) for k in m.markers], np.float32).reshape(-1, 2)

    def on_plan(self, m):
        self.plan = np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses], np.float32).reshape(-1, 2)

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.translation.x, t.translation.y, yaw
        except Exception:
            return float('nan'), float('nan'), float('nan')

    def grid(self):
        i = self.map.info
        g = np.asarray(self.map.data, dtype=np.int8).reshape(i.height, i.width)
        return g, i.resolution, i.origin.position.x, i.origin.position.y

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0s is None:
            self.t0s = now
        t = now - self.t0s
        x, y, yaw = self.pose()
        known = cov = 0.0
        if self.map is not None:
            g, r, ox, oy = self.grid()
            known = float(np.count_nonzero(g >= 0)) * r * r
            cov = world_raster.coverage(g, r, ox, oy, self.px, self.py)
            if self.a.frames and t - self.last_frame >= 2.0:
                self.last_frame = t
                self.frames.append(dict(t=t, map=g.copy(), res=r, ox=ox, oy=oy, pose=(x, y, yaw),
                                        frontiers=self.frontiers, cands=self.cands, plan=self.plan[::3]))
        if self.last_gain is None or known > self.best + 0.2:
            self.best, self.last_gain = known, t
        if not self.done and t > 20:
            reason = 'coverage' if cov >= self.a.stop else 'stall' if t - self.last_gain > self.a.stall else None
            if reason:
                self.done = True
                with open(os.path.join(self.a.out, 'done.txt'), 'w') as f:
                    f.write(f'{reason} {t:.1f}\n')
        self.w.writerow([round(t, 1), round(time.time() - self.t0w, 1), round(known, 2), round(cov, 4),
                         round(x, 3), round(y, 3), round(self.dist, 2)])
        self.f.flush()

    def save(self):
        if self.map is not None:
            g, r, ox, oy = self.grid()
            np.savez_compressed(os.path.join(self.a.out, 'final_map.npz'), data=g, res=r, ox=ox, oy=oy)
        if self.frames:
            F = self.frames
            np.savez_compressed(
                os.path.join(self.a.out, 'frames.npz'),
                t=np.array([f['t'] for f in F]), pose=np.array([f['pose'] for f in F]),
                meta=np.array([(f['res'], f['ox'], f['oy'], *f['map'].shape) for f in F]),
                maps=np.concatenate([f['map'].ravel() for f in F]),
                **{f'{k}_{i}': f[k] for i, f in enumerate(F) for k in ('frontiers', 'cands', 'plan')})


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True)
    p.add_argument('--world', required=True)
    p.add_argument('--stall', type=float, default=150.0)
    p.add_argument('--stop', type=float, default=0.99)
    p.add_argument('--frames', action='store_true')
    a = p.parse_args()
    rclpy.init()
    n = Monitor(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n.save()
        n.f.close()


if __name__ == '__main__':
    main()
