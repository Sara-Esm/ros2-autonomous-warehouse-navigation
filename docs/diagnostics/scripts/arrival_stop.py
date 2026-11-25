#!/usr/bin/env python3
"""Motion around each ARRIVED event: last nonzero commands, last true motion, marker confirm.

Usage: arrival_stop.py <run_dir>
"""
import re, sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
S = {'/odom': [], '/cmd_vel': [], '/cmd_vel_nav': []}
while r.has_next():
    t, d, ts = r.read_next()
    if t in S:
        m = deserialize_message(d, ty[t]); tw = m.twist.twist if t == '/odom' else m
        S[t].append((ts * 1e-9, tw.linear.x, tw.angular.z))
od, cv, cn = (np.array(S[k]) for k in ('/odom', '/cmd_vel', '/cmd_vel_nav'))
ev = {}
for l in open(run + '/mission.log'):
    m = re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (ARRIVED|TASK COMPLETE|Correct marker)', l)
    if m:
        ev.setdefault(m.group(2), []).append(float(m.group(1)))
print('Times are wall seconds relative to the ARRIVED log line (bag receipt times).')
for a, c, tc in zip(ev['ARRIVED'], ev['Correct marker'], ev['TASK COMPLETE']):
    def last(arr, tv, tw):
        s = arr[(arr[:, 0] > a - 5) & (arr[:, 0] <= tc) & ((np.abs(arr[:, 1]) > tv) | (np.abs(arr[:, 2]) > tw))]
        return s[-1, 0] - a if len(s) else float('nan')
    w = od[(od[:, 0] > a) & (od[:, 0] <= tc)]
    print(f'last nonzero DWB cmd {last(cn,1e-4,1e-4):+.2f}s, smoother {last(cv,1e-4,1e-4):+.2f}s, true motion(>2mm/s,>0.005rad/s) '
          f'{last(od,0.002,0.005):+.2f}s; marker confirm {c-a:+.2f}s; TASK COMPLETE {tc-a:+.2f}s; '
          f'max true |v| after {np.abs(w[:,1]).max()*1000:.1f} mm/s, |w| {np.abs(w[:,2]).max():.3f} rad/s')
