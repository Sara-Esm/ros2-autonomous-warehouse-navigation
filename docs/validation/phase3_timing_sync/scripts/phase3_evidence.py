#!/usr/bin/env python3
"""Phase 3 evidence additions: pooled ack-latency estimates, per-goal timeout rates, host conditions.

Usage: phase3_evidence.py
"""
import re, bisect
from collections import defaultdict
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
RUNS = [('Phase 1', 'diagnostics_phase1/run'), ('Phase 2a', 'diagnostics_phase2/run'),
        ('Phase 2b', 'diagnostics_phase2b/run'), ('Phase 3', 'diagnostics_phase3/run')]
ACTIONS = ('compute_path_to_pose', 'follow_path', 'spin', 'backup', 'wait')
pooled = []; per = {}
for lab, run in RUNS:
    meta = open(run + '/meta.txt').read()
    ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1)); me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    sends, seen, acc = [], set(), []; goals = defaultdict(set)
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/behavior_tree_log':
            for e in deserialize_message(d, ty[t]).event_log:
                if e.node_name == 'ComputePathToPose' and e.previous_status == 'IDLE' and e.current_status == 'RUNNING':
                    sends.append(e.timestamp.sec + e.timestamp.nanosec * 1e-9)
        elif t.endswith('/_action/status'):
            a = t.split('/')[1]
            for s in deserialize_message(d, ty[t]).status_list:
                g = bytes(s.goal_info.goal_id.uuid).hex()
                if ms <= ts * 1e-9 <= me + 1: goals[a].add(g)
                if a == 'compute_path_to_pose' and g not in seen:
                    seen.add(g); acc.append(ts * 1e-9)
    sends = sorted(set(round(x, 4) for x in sends)); lat = []
    for a in acc:
        i = bisect.bisect_right(sends, a) - 1
        if i >= 0 and a - sends[i] < 2.0: lat.append((a - sends[i]) * 1e3)
    pooled += lat
    log = open(run + '/launch.log', errors='ignore').read().splitlines()
    to = defaultdict(int)
    for l in log:
        m = re.search(r'\[(\d{10}\.\d+)\].*acknowledge goal request for (\w+)', l)
        if m and ms <= float(m.group(1)) <= me: to[m.group(2)] += 1
    per[lab] = (lat, to, {a: len(goals[a]) for a in ACTIONS}, (me - ms) / 60)
lat = np.array(pooled); q = lambda p: np.percentile(lat, p)
print(f'## Estimated compute_path_to_pose acknowledgement latency, pooled over {len(lat)} goals (Phases 1, 2a, 2b, 3)')
print(f'p50 {q(50):.1f} ms, p90 {q(90):.1f}, p95 {q(95):.1f}, p99 {q(99):.1f}, p99.9 {q(99.9):.1f}, max {lat.max():.1f} ms; '
      f'> 20 ms: {np.sum(lat>20)} ({100*np.mean(lat>20):.1f}%), > 50 ms: {np.sum(lat>50)}, > 100 ms: {np.sum(lat>100)}, > 200 ms: {np.sum(lat>200)}')
print('(estimate = status-topic receipt time minus BT send-event time; includes status transport; not a direct response measurement)')
print('\n## Acknowledgement timeouts per goal and per wall minute (matching action categories)')
print('| Run | Action | Goals | Timeouts | Per goal | Per wall min | Estimated ack > 20 ms (planner) |')
print('|---|---|---|---|---|---|---|')
for lab, (l, to, g, mins) in per.items():
    for a in ACTIONS:
        if g[a] or to[a]:
            extra = f'{int(np.sum(np.array(l)>20))} of {len(l)}' if a == 'compute_path_to_pose' else ''
            print(f'| {lab} | {a} | {g[a]} | {to[a]} | {to[a]/max(g[a],1):.3f} | {to[a]/mins:.2f} | {extra} |')
print('\n## Host conditions during the mission window (top samples every ~2 s)')
for lab, run in RUNS[2:]:
    meta = open(run + '/meta.txt').read()
    ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1)); me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
    txt = open(run + '/cpu_top.log', errors='ignore').read().split('=== ')[1:]
    idle, load, used, avail, proc = [], [], [], [], defaultdict(list)
    for b in txt:
        t = float(b.split('\n', 1)[0])
        if not ms <= t <= me: continue
        m = re.search(r'([\d.]+) id', b); idle.append(float(m.group(1))) if m else None
        m = re.search(r'load average: ([\d.]+)', b); load.append(float(m.group(1))) if m else None
        m = re.search(r'MiB Mem :\s+[\d.]+ total,\s+[\d.]+ free,\s+([\d.]+) used,\s+[\d.]+ buff/cache', b); used.append(float(m.group(1))) if m else None
        m = re.search(r'([\d.]+) avail Mem', b); avail.append(float(m.group(1))) if m else None
        for l in b.splitlines():
            f = l.split()
            if len(f) >= 12 and f[0].isdigit():
                try: proc[f[11]].append(float(f[8]))
                except ValueError: pass
    n = len(idle)
    top = sorted(((sum(v) / n, k) for k, v in proc.items()), reverse=True)[:5]
    print(f'{lab}: {n} samples; CPU busy median {100-np.median(idle):.0f}%; load avg 1-min median {np.median(load):.1f}; '
          f'memory used median {np.median(used):.0f} MiB, available min {min(avail):.0f} MiB; top processes (mean % of one core): '
          + ', '.join(f'{k} {v:.0f}' for v, k in top))
