#!/usr/bin/env python3
"""Pair BT ComputePathToPose/FollowPath FAILURE records with acknowledgement timeouts.

Usage: phase1_reconcile.py <run_dir>
Note: the stock BT has RecoveryNode wrappers with the same names as the action nodes.
"""
import re, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run = sys.argv[1]
meta = open(run + '/meta.txt').read()
ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1))
me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
lines = open(run + '/launch.log', errors='ignore').read().splitlines()
acks = [(float(m.group(1)) - ms, m.group(2)) for l in lines
        for m in [re.search(r'\[(\d{10}\.\d+)\].*acknowledge goal request for (\w+)', l)] if m]
aborts = [float(m.group(1)) - ms for l in lines
          for m in [re.search(r'\[(\d{10}\.\d+)\].*\[follow_path\] \[ActionServer\] Aborting handle', l)] if m]
print(f'ack timeouts in whole launch log: {len(acks)}; inside mission window: {sum(0 <= a <= me - ms for a, _ in acks)}')
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
ev = []
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/behavior_tree_log':
        for e in deserialize_message(d, ty[t]).event_log:
            if e.current_status == 'FAILURE' and e.node_name in ('ComputePathToPose', 'FollowPath', 'NavigateWithReplanning'):
                ev.append((ts * 1e-9 - ms, e.node_name))
fails = sorted(set((round(t, 1), n) for t, n in ev if n != 'NavigateWithReplanning'))
want = {'ComputePathToPose': 'compute_path_to_pose', 'FollowPath': 'follow_path'}
print(f'distinct (0.1 s-rounded) ComputePathToPose/FollowPath FAILURE events: {len(fails)}')
used = {}
for t, n in fails:
    m = [(a, k) for a, k in acks if k == want[n] and -0.1 <= t - a <= 1.0]
    used.setdefault(max(m)[0] if m else None, []).append((t, n))
for a, fl in sorted(used.items(), key=lambda kv: (kv[0] is None, kv[0])):
    tag = '  <-- one timeout, two FAILURE records' if len(fl) > 1 else ''
    print(f'  timeout {a} -> ' + ', '.join(f'{n} FAILURE at {t:.1f} s' for t, n in fl) + tag)
print('NavigateWithReplanning FAILURE:', sorted(set(round(t, 1) for t, n in ev if n == 'NavigateWithReplanning')))
print(f'"[follow_path] [ActionServer] Aborting handle" warnings: {len(aborts)} at {[round(a, 1) for a in aborts]}')
