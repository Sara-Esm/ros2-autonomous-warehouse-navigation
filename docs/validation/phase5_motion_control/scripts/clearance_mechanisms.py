#!/usr/bin/env python3
"""Clearance mechanisms: south pinch (mid-route) vs final station alignment. Run via eval_wrapper.py.

Usage: clearance_mechanisms.py <label>=<run_dir> [...]
Reference geometry: diagnostics_20261125/static/gt_body.npy (true geometry at body height, 0.05 m cells,
origin (-7.56, -10.8)); distances +-0.025 m quantization.
- CENTRE clearance: distance from the robot centre (base_footprint, wheel axle; truth /odom) to geometry.
- FOOTPRINT clearance: minimum over the perimeter of the robot's collision outline (turtlebot3_burger
  model.sdf: base box x -0.102..0.038, y +-0.070; wheels x +-0.033 at y +-0.071..0.089), placed with the
  truth pose; points inside geometry count as 0. Orientation-dependent.
- PINCH window: samples within 0.5 m of world (0.45, -8.87) (location of the Dispatch-leg minimum in all
  baseline and validation missions), while moving (|v| >= 0.05).
  For the pinch minimum: the latest /plan (map frame -> world with the evaluation fit) is used to get the
  PLAN's own clearance at the closest plan point, the robot's signed offset from the plan (+ = toward lower
  clearance), the heading relative to the plan direction there, and the AMCL position error at that time.
- FINAL ALIGNMENT window: within 0.30 m of the leg's final stop (station approach incl. final rotation).
"""
import math, re, sys
import cv2, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, 'src/warehouse_bringup')
from warehouse_bringup import stations as st  # noqa: E402  (eval_wrapper attaches map_to_world)
gt = np.load('diagnostics_20261125/static/gt_body.npy') > 0; GH, GW = gt.shape
dgt = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * 0.05
PX, PY = 0.45, -8.87


def dist(x, y):
    c = np.clip(np.floor((np.asarray(x) + 7.56) / 0.05).astype(int), 0, GW - 1)
    r = GH - 1 - np.clip(np.floor((np.asarray(y) + 10.8) / 0.05).astype(int), 0, GH - 1)
    return dgt[r, c]


# collision outline perimeter in the body frame (2 cm spacing)
pts = []
for x in np.arange(-0.102, 0.0381, 0.02):
    pts += [(x, 0.070), (x, -0.070)]
for y in np.arange(-0.070, 0.0701, 0.02):
    pts += [(-0.102, y), (0.038, y)]
for s in (1, -1):
    for x in np.arange(-0.033, 0.0331, 0.02):
        pts += [(x, s * 0.089)]
    for y in np.arange(0.071, 0.0891, 0.009):
        pts += [(-0.033, s * y), (0.033, s * y)]
PTS = np.array(pts)


def footprint_clear(x, y, th):
    c, s = math.cos(th), math.sin(th)
    wx = x + PTS[:, 0] * c - PTS[:, 1] * s; wy = y + PTS[:, 0] * s + PTS[:, 1] * c
    return float(dist(wx, wy).min())


for arg in sys.argv[1:]:
    lab, run = arg.split('=', 1)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    od, plans, amcl = [], [], []
    while r.has_next():
        t, d, ts = r.read_next(); w = ts * 1e-9
        if t == '/odom':
            m = deserialize_message(d, ty[t]); q = m.pose.pose.orientation
            od.append((w, m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z), m.twist.twist.linear.x))
        elif t == '/plan':
            m = deserialize_message(d, ty[t])
            if m.poses:
                plans.append((w, np.array([st.map_to_world(p.pose.position.x, p.pose.position.y, 0.0)[:2] for p in m.poses])))
        elif t == '/amcl_pose':
            m = deserialize_message(d, ty[t]); p = m.pose.pose.position
            amcl.append((w,) + tuple(st.map_to_world(p.x, p.y, 0.0)[:2]))
    od = np.array(od); amcl = np.array(amcl); pw = [p[0] for p in plans]
    ev = [(float(m.group(1)), m.group(2), m.group(3)) for l in open(run + '/mission.log')
          for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)(.*)', l)] if m]
    legs = [(n0.split(':')[0].strip(), w0, w1) for (w0, k0, n0), (w1, k1, _) in zip(ev, ev[1:]) if k0 == 'Navigating to' and k1 == 'ARRIVED']
    print(f'{lab}:')
    for name, w0, w1 in legs:
        o = od[(od[:, 0] >= w0) & (od[:, 0] <= w1)]
        cen = dist(o[:, 1], o[:, 2]); fx, fy = o[-1, 1], o[-1, 2]
        near_final = np.hypot(o[:, 1] - fx, o[:, 2] - fy) <= 0.30
        pinch = (np.hypot(o[:, 1] - PX, o[:, 2] - PY) <= 0.5) & (np.abs(o[:, 4]) >= 0.05)
        fpc = np.array([footprint_clear(*o[k, 1:4]) for k in range(len(o))])
        line = f'  {name:28s} final-alignment: centre {cen[near_final].min():.2f} / footprint {fpc[near_final].min():.2f} m'
        if pinch.any():
            k = np.nonzero(pinch)[0][np.argmin(cen[pinch])]; w, x, y, th = o[k, :4]
            j = int(np.searchsorted(pw, w)) - 1; pl = plans[j][1]
            di = np.hypot(pl[:, 0] - x, pl[:, 1] - y); i = int(np.argmin(di))
            pc = float(dist(pl[i, 0], pl[i, 1])); a, b = pl[max(i - 3, 0)], pl[min(i + 3, len(pl) - 1)]
            ph = math.atan2(b[1] - a[1], b[0] - a[0]); dh = math.degrees((th - ph + math.pi) % (2 * math.pi) - math.pi)
            ae = float(np.hypot(*(amcl[np.argmin(np.abs(amcl[:, 0] - w)), 1:3] - (x, y)))) if len(amcl) else float('nan')
            line += (f' | PINCH: centre {cen[k]:.2f} / footprint {footprint_clear(x, y, th):.2f} m (window min footprint {fpc[pinch].min():.2f})'
                     f'; plan clearance {pc:.2f} m, robot off plan {di[i]*100:.1f} cm ({"toward" if cen[k] < pc else "away from"} obstacle)'
                     f', heading vs plan {dh:+.1f} deg, AMCL error {ae*100:.1f} cm, at ({x:+.2f},{y:+.2f})')
        print(line)
