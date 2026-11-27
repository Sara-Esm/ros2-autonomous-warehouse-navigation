#!/usr/bin/env python3
"""Evaluate replayed local costmaps (three configurations) against the recorded robot motion.

Usage: local_unknown_eval.py <replay_dir> <run_dir>
Per snapshot while moving (|v| >= 0.05 m/s): (a) unknown share of the 3x3 m window; (b) cell under the robot
unknown; (c) EXECUTED-PATH check: robot centre over the next 1.5 s sim (DWB sim_time) crosses an unknown cell
(DWB BaseObstacle would reject it); (d) FAN check: of 21 constant-curvature arcs (v 0.22 m/s, w -1..+1 rad/s,
1.5 s) how many avoid unknown (-1), inscribed (99) and lethal (100) cells at the centre; blocking = 0 arcs.
Initialization: unknown share during the first 10 s of costmap output.
"""
import math, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
rd, run = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}; od = []
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/odom':
        m = deserialize_message(d, ty[t]); q = m.pose.pose.orientation
        od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                   math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z), m.twist.twist.linear.x))
od = np.array(od)
W = np.linspace(-1.0, 1.0, 21); TT = np.linspace(0.05, 1.5, 30)
for cfg in ('lc_current', 'lc_track', 'lc_track_inf'):
    z = np.load(f'{rd}/{cfg}.npz'); meta = z[f'{cfg}_meta']; data = z[f'{cfg}_data']
    t0 = meta[0, 0]; init = []; unk = []; under = []; ex_bad = []; fan_valid = []; block = 0; moving = 0
    for (ts, ox, oy, w_, h_), g in zip(meta, data):
        w_, h_ = int(w_), int(h_); g = g.reshape(h_, w_)
        if ts - t0 <= 10: init.append(np.mean(g == -1))
        k = np.searchsorted(od[:, 0], ts)
        if k >= len(od) or abs(od[k, 4]) < 0.05: continue
        moving += 1; x, y, th = od[k, 1:4]
        def cell(px, py):
            cx, cy = int(math.floor((px - ox) / 0.05)), int(math.floor((py - oy) / 0.05))
            return g[cy, cx] if 0 <= cx < w_ and 0 <= cy < h_ else None
        unk.append(np.mean(g == -1)); under.append(cell(x, y) == -1)
        fut = od[(od[:, 0] > ts) & (od[:, 0] <= ts + 1.5)]
        ex_bad.append(any(cell(px, py) == -1 for px, py in fut[:, 1:3]))
        ok = 0
        for w in W:
            good = True
            for tt in TT:
                if abs(w) < 1e-6: px, py = x + 0.22 * tt * math.cos(th), y + 0.22 * tt * math.sin(th)
                else:
                    R = 0.22 / w; px = x + R * (math.sin(th + w * tt) - math.sin(th)); py = y - R * (math.cos(th + w * tt) - math.cos(th))
                c = cell(px, py)
                if c is None: break
                if c == -1 or c >= 99: good = False; break
            ok += good
        fan_valid.append(ok); block += ok == 0
    fv = np.array(fan_valid)
    print(f'{cfg}: {len(meta)} snapshots, {moving} while moving')
    print(f'   initialization (first 10 s): unknown share median {np.median(init)*100:.0f}%')
    print(f'   moving: unknown share of 3x3 m window median {np.median(unk)*100:.0f}%, p95 {np.percentile(unk,95)*100:.0f}%; cell under robot unknown {np.mean(under)*100:.1f}%')
    print(f'   executed next-1.5 s path crosses an unknown cell (DWB would reject it): {np.mean(ex_bad)*100:.1f}% of moving snapshots')
    print(f'   fan arcs valid (of 21): median {np.median(fv):.0f}, p5 {np.percentile(fv,5):.0f}; snapshots with NO valid arc (blocking): {block} ({block/max(moving,1)*100:.1f}%)')
