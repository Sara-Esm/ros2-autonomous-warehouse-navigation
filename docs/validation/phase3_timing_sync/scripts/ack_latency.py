#!/usr/bin/env python3
"""Estimate compute_path_to_pose goal-acknowledgement latency from a recorded mission.

Usage: ack_latency.py <label>=<run_dir> [...]
latency ~= (bag receipt time of the first status message listing a new goal UUID)
           - (BT event time of the last preceding ComputePathToPose IDLE->RUNNING transition).
Both are wall-clock; status transport adds a small positive bias. Goals the BT abandoned (ack
timeout) are still accepted later by the server and appear with their late acceptance time.
"""
import bisect, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for arg in sys.argv[1:]:
    label, run = arg.split('=', 1)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    if '/compute_path_to_pose/_action/status' not in ty:
        print(f'{label}: no action status recorded'); continue
    sends, seen, acc = [], set(), []
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/behavior_tree_log':
            for e in deserialize_message(d, ty[t]).event_log:
                if e.node_name == 'ComputePathToPose' and e.previous_status == 'IDLE' and e.current_status == 'RUNNING':
                    sends.append(e.timestamp.sec + e.timestamp.nanosec * 1e-9)
        elif t == '/compute_path_to_pose/_action/status':
            for s in deserialize_message(d, ty[t]).status_list:
                g = bytes(s.goal_info.goal_id.uuid).hex()
                if g not in seen:
                    seen.add(g); acc.append(ts * 1e-9)
    sends = sorted(set(round(x, 4) for x in sends))
    lat = []
    for a in acc:
        i = bisect.bisect_right(sends, a) - 1
        if i >= 0 and a - sends[i] < 2.0:
            lat.append(a - sends[i])
    lat = np.array(lat) * 1e3
    q = lambda p: np.percentile(lat, p)
    print(f'{label}: n={len(lat)} goals; latency ms: p50 {q(50):.1f}, p90 {q(90):.1f}, p95 {q(95):.1f}, p99 {q(99):.1f}, '
          f'p99.9 {q(99.9):.1f}, max {lat.max():.1f}; > 20 ms: {np.sum(lat>20)} ({100*np.mean(lat>20):.1f}%), '
          f'> 50 ms: {np.sum(lat>50)}, > 100 ms: {np.sum(lat>100)}, > 200 ms: {np.sum(lat>200)}')
