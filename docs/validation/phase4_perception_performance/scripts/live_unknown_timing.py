#!/usr/bin/env python3
"""How much of the executed 'live-unknown' traversal is a 1 Hz snapshot artefact? (run via eval_wrapper.py)

Usage: live_unknown_timing.py <run_dir> <workspace>
Same definition as unknown_traversal_v2.py (robot CENTRE from truth /odom mapped to the map frame with the
evaluation fit; cell lookup in the latest /global_costmap/costmap), but snapshots are ordered by their sim
header stamps (costmap update time) against odom sim stamps. For every odom segment flagged live-unknown:
snapshot age (sim), and the cell's value in the NEXT snapshot. Sensitivity: the same total with the robot
position shifted by one cell (5 cm) in each of 8 directions (alignment uncertainty of the fit).
"""
import bisect, re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run, ws = sys.argv[1], sys.argv[2]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup import stations as st  # noqa: E402
meta = open(run + '/meta.txt').read()
ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1)); me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
odom, cms = [], []
while r.has_next():
    t, d, ts = r.read_next(); w = ts * 1e-9
    if t == '/odom' and ms <= w <= me:
        m = deserialize_message(d, ty[t]); odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y))
    elif t == '/global_costmap/costmap':
        m = deserialize_message(d, ty[t]); i = m.info
        cms.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, i.width, i.height, i.origin.position.x, i.origin.position.y,
                    np.array(m.data, dtype=np.int8).reshape(i.height, i.width)))
cms.sort(key=lambda c: c[0]); cs = [c[0] for c in cms]
odom = np.array(odom); mxy = np.array([st.world_to_map(x, y, 0.0)[:2] for _, x, y in odom])
gaps = np.diff(cs); print(f'global costmap snapshots: {len(cms)}; interval (sim header) median {np.median(gaps):.2f} s, p95 {np.percentile(gaps,95):.2f}, max {gaps.max():.2f}')
def val(i, x, y):
    _, cw, ch, ox, oy, g = cms[i]
    return g[min(max(int(np.floor((y - oy) / 0.05)), 0), ch - 1), min(max(int(np.floor((x - ox) / 0.05)), 0), cw - 1)]
def total(shift=(0.0, 0.0), detail=False):
    tot = 0.0; ages = []; nxt = {'free': 0.0, 'unknown': 0.0, 'other': 0.0, 'none': 0.0}
    for k in range(1, len(odom)):
        i = bisect.bisect_right(cs, odom[k, 0]) - 1
        if i < 0: continue
        x, y = mxy[k] + shift
        if val(i, x, y) != -1: continue
        seg = float(np.hypot(*(mxy[k] - mxy[k - 1]))); tot += seg
        if detail:
            ages.append((odom[k, 0] - cs[i], seg))
            if i + 1 < len(cms):
                v = val(i + 1, x, y); nxt['free' if v == 0 else 'unknown' if v == -1 else 'other'] += seg
            else: nxt['none'] += seg
    return tot, ages, nxt
tot, ages, nxt = total(detail=True)
a = np.array(ages)
print(f'executed on live-unknown (sim-stamp ordering): {tot:.2f} m')
if len(a):
    w = a[:, 1]
    print(f'  snapshot age of those segments (sim): length-weighted median {np.median(np.repeat(a[:,0], np.maximum(1,(w*1000).astype(int)))):.2f} s, max {a[:,0].max():.2f} s')
print('  same cell in the NEXT snapshot: ' + ', '.join(f'{k} {v:.2f} m' for k, v in nxt.items()))
sh = [total((dx, dy))[0] for dx in (-0.05, 0, 0.05) for dy in (-0.05, 0, 0.05) if (dx, dy) != (0, 0)]
print(f'  one-cell (5 cm) alignment sensitivity: {min(sh):.2f} .. {max(sh):.2f} m')
