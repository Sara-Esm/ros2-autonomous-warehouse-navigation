#!/usr/bin/env python3
"""Per-arrival accuracy for route sessions and missions (generic target lookup by logged station name).

Usage: session_eval.py <label>=<run_dir> [...]
Arrivals: navigate_to_pose SUCCEEDED (bag receipt), in order, matched to the logged 'Navigating to <name>'
targets. Settled: first time after acceptance from which |v| < 2 mm/s and |w| < 0.005 rad/s hold for 1.0 s
sim (same definition as arrival_accuracy.py). Errors vs the intended physical approach pose (world frame).
Approach-frame offset of the settled stop: dx (+ = toward the marker), dy; the camera envelope failed only
for dx >= +0.125 m (Phase 4 report §5.1).
Excludes the first leg (from the spawn) from the station summary only if --skip-first is given.
"""
import math, re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
INTENDED = {'Home Charging Dock': (-3.678, 9.3, 90.0), 'Return to Home Charging Dock': (-3.678, 9.3, 90.0),
            'Loading Zone': (1.803, 9.3, 90.0), 'Inventory Scan Station': (1.801, -9.0, -90.0),
            'Dispatch Zone': (-3.630, -9.3, -90.0)}
wrap = lambda a: (a + 180) % 360 - 180
for arg in [a for a in sys.argv[1:] if '=' in a]:
    lab, run = arg.split('=', 1)
    names = [m.group(1) for l in open(run + '/mission.log')
             for m in [re.search(r'\[warehouse_autonomy\]: Navigating to ([^:]+):', l)] if m]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    od, succ, seen = [], [], set()
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/odom':
            m = deserialize_message(d, ty[t]); q = m.pose.pose.orientation
            od.append((ts * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                       m.pose.pose.position.y, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))),
                       m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif t == '/navigate_to_pose/_action/status':
            for s in deserialize_message(d, ty[t]).status_list:
                g = bytes(s.goal_info.goal_id.uuid).hex()
                if s.status == 4 and g not in seen:
                    seen.add(g); succ.append(ts * 1e-9)
    od = np.array(od); succ = sorted(succ); src = 'navigate_to_pose SUCCEEDED status'
    if not succ:   # bag without hidden action-status topics: use the ARRIVED log time (logged on result receipt)
        succ = [float(m.group(1)) for l in open(run + '/mission.log')
                for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: ARRIVED', l)] if m]
        src = "'ARRIVED' log time (status topics not recorded)"
    still = (np.abs(od[:, 5]) < 0.002) & (np.abs(od[:, 6]) < 0.005)
    rows = []
    for k, w in enumerate(succ):
        name = names[k]; ix, iy, iyaw = INTENDED[name]
        i = np.searchsorted(od[:, 0], w); a = od[min(i, len(od) - 1)]
        j, settled = i, None
        while j < len(od):
            if still[j]:
                e = j
                while e < len(od) and still[e] and od[e, 1] - od[j, 1] < 1.0:
                    e += 1
                if e < len(od) and od[e, 1] - od[j, 1] >= 1.0:
                    settled = j; break
                j = e
            j += 1
        s = od[settled] if settled is not None else od[-1]
        th = math.radians(iyaw); ex, ey = s[2] - ix, s[3] - iy
        dx = ex * math.cos(th) + ey * math.sin(th); dy = -ex * math.sin(th) + ey * math.cos(th)
        rows.append((name, math.hypot(a[2] - ix, a[3] - iy), wrap(a[4] - iyaw), abs(a[6]), s[1] - a[1],
                     math.hypot(ex, ey), wrap(s[4] - iyaw), dx, dy))
    print(f'{lab}: {len(rows)} arrivals (acceptance from {src})')
    for nm, pa, ha, wa, tset, ps, hs, dx, dy in rows:
        print(f'  {nm:30s} accept {pa:.3f} m / {ha:+5.1f} deg (|w| {wa:.3f})  settled after {tset:4.2f} s  settled {ps:.3f} m / {hs:+5.1f} deg  dx {dx:+.3f} dy {dy:+.3f}')
    hs = np.array([abs(x[6]) for x in rows]); ps = np.array([x[5] for x in rows]); dxs = np.array([x[7] for x in rows])
    print(f'  SUMMARY settled |heading| mean {hs.mean():.1f} deg, max {hs.max():.1f} deg; settled position mean {ps.mean():.3f} m, max {ps.max():.3f} m; '
          f'dx range {dxs.min():+.3f} .. {dxs.max():+.3f} m; settle time mean {np.mean([x[4] for x in rows]):.2f} s')
