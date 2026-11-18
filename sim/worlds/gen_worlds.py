#!/usr/bin/env python3
"""Generate the Gazebo test worlds (SDF) used in simulation.

    python3 gen_worlds.py [out_dir]

Worlds (walls are 0.1 m thick, 1 m high boxes; obstacles are boxes/cylinders):
  square          empty 6 x 6 m room
  corridor        14 x 1.5 m corridor with parallel walls
  corridor_taper  14 m corridor whose long walls diverge by 5 deg (1.5 m -> ~2.7 m)
  four_rooms      8 x 8 m arena split into four 4 x 4 m rooms connected in a ring by 1 m doors
  three_rooms     one large room with obstacles of various shapes + two smaller rooms,
                  wall niches and short dead ends (doors / dead ends >= 1.1 m)
"""
import math
import os
import sys

T = 0.1   # wall thickness [m]
H = 1.0   # wall height [m]


def box(cx, cy, sx, sy, yaw=0.0):
    return (cx, cy, sx, sy, yaw)


def cyl(cx, cy, r):
    return ('cyl', cx, cy, r)


def rect_room(x0, x1, y0, y1):
    """Four walls around the interior [x0, x1] x [y0, y1]."""
    w, h = x1 - x0, y1 - y0
    return [box((x0 + x1) / 2, y1 + T / 2, w + 2 * T, T), box((x0 + x1) / 2, y0 - T / 2, w + 2 * T, T),
            box(x0 - T / 2, (y0 + y1) / 2, T, h), box(x1 + T / 2, (y0 + y1) / 2, T, h)]


def hseg(x0, x1, y):
    return box((x0 + x1) / 2, y, x1 - x0, T)


def vseg(x, y0, y1):
    return box(x, (y0 + y1) / 2, T, y1 - y0)


def seg(x0, y0, x1, y1):
    """Wall on the line (x0,y0)-(x1,y1), extended by T/2 at both ends to close corners."""
    L = math.hypot(x1 - x0, y1 - y0)
    a = math.atan2(y1 - y0, x1 - x0)
    return box((x0 + x1) / 2, (y0 + y1) / 2, L + T, T, a)


def poly(pts):
    """Walls along a closed polyline."""
    return [seg(*pts[k], *pts[(k + 1) % len(pts)]) for k in range(len(pts))]


def taper_corridor(x0, x1, w0, ang_deg):
    """Corridor with long walls diverging symmetrically by ang_deg: width w0 at x0, larger at x1."""
    a = math.radians(ang_deg / 2)
    L = x1 - x0
    h0 = w0 / 2
    h1 = h0 + L * math.tan(a)
    ln = L / math.cos(a) + 2 * T
    xm, ym = (x0 + x1) / 2, (h0 + h1) / 2
    off = T / 2 / math.cos(a)
    return [box(xm, ym + off, ln, T, a), box(xm, -ym - off, ln, T, -a),
            box(x0 - T / 2, 0, T, 2 * h0 + 2 * T), box(x1 + T / 2, 0, T, 2 * h1 + 2 * T)]


WORLDS = {}
WORLDS['square'] = rect_room(-3, 3, -3, 3)
WORLDS['corridor'] = rect_room(-7, 7, -0.75, 0.75)
WORLDS['corridor_taper'] = taper_corridor(-7, 7, 1.5, 5.0)

_four = rect_room(-4, 4, -4, 4)
_four += [vseg(0, -4, -2.5), vseg(0, -1.5, 1.5), vseg(0, 2.5, 4),
          hseg(-4, -2.5, 0), hseg(-1.5, 1.5, 0), hseg(2.5, 4, 0)]
WORLDS['four_rooms'] = _four

_outer = [(-5.5, -4), (-1.6, -4), (-1.6, -4.6), (-0.4, -4.6), (-0.4, -4),   # south niche (large room)
          (5.5, -4), (5.5, -3.0), (6.3, -3.0), (6.3, -1.9), (5.5, -1.9),    # east niche (room C)
          (5.5, 4), (3.3, 4), (3.3, 5.8), (2.2, 5.8), (2.2, 4),             # north dead end (room B)
          (-3.3, 4), (-3.3, 4.8), (-4.4, 4.8), (-4.4, 4),                   # north niche (large room)
          (-5.5, 4), (-5.5, 0.6), (-7.3, 0.6), (-7.3, -0.5), (-5.5, -0.5)]  # west dead end (large room)
_three = poly(_outer)
_three += [seg(1.0, -4, 1.0, -2.8), seg(1.0, -1.7, 1.0, 1.7), seg(1.0, 2.8, 1.0, 4)]  # large room | B, C (2 doors)
_three += [seg(1.0, 0.3, 3.6, 0.3), seg(4.7, 0.3, 5.5, 0.3)]                          # B | C (1 door)
_three += [seg(2.5, -4, 2.5, -2.6)]                                                   # niche in room C
_three += [cyl(-3.6, 2.2, 0.30), cyl(-1.0, -2.4, 0.45),                               # large room obstacles
           box(-2.2, -0.6, 1.4, 0.3, math.radians(35)),
           box(-0.6, 1.3, 1.2, 0.3), box(-1.05, 1.75, 0.3, 1.2),                     # L shape
           box(-4.3, -2.4, 0.7, 0.7, math.radians(45)),                               # diamond
           box(-3.6, 0.0, 1.0, 0.25), box(-3.6, 0.0, 0.25, 1.0),                     # cross
           cyl(3.0, 2.0, 0.25), box(4.0, 2.9, 0.9, 0.3, math.radians(-25)),           # room B
           box(3.8, -1.3, 1.0, 0.5, math.radians(-20)), cyl(2.0, -1.0, 0.35)]         # room C
WORLDS['three_rooms'] = _three


def link(i, b):
    if b[0] == 'cyl':
        _, cx, cy, r = b
        return f"""      <link name="w{i}"><pose>{cx:.4f} {cy:.4f} {H/2} 0 0 0</pose>
        <collision name="c"><geometry><cylinder><radius>{r:.3f}</radius><length>{H}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{r:.3f}</radius><length>{H}</length></cylinder></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>"""
    cx, cy, sx, sy, yaw = b
    return f"""      <link name="w{i}"><pose>{cx:.4f} {cy:.4f} {H/2} 0 0 {yaw:.6f}</pose>
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {H}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {H}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>"""


def world_sdf(name, shapes):
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <scene><shadows>false</shadows></scene>
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <model name="{name}_walls">
      <static>true</static>
{chr(10).join(link(i, b) for i, b in enumerate(shapes))}
    </model>
  </world>
</sdf>
"""


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    for n, s in WORLDS.items():
        with open(os.path.join(out, f'{n}.world'), 'w') as f:
            f.write(world_sdf(n, s))
        print(n, len(s), 'shapes')
