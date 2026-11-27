#!/usr/bin/env python3
"""
Timing and action-sequencing metrics for a recorded mission (Phase 3 comparison).

Usage: timing_metrics.py <label>=<run_dir> [...]
- ack timeouts per action, per wall minute; service-call timeouts; halt cancel/result errors;
  planner "failed to create plan".
- late-accepted (orphaned) goals: after a BT ack timeout for action A at time w, a new goal of A first
  appearing in A's status within 1.0 s after w (the server accepted a goal the BT had abandoned).
- overlapping behaviours: time during which two of spin/backup/wait/drive_on_heading are EXECUTING at once.
- update rates during navigation: /cmd_vel_nav (controller), /cmd_vel (velocity smoother), /plan (planner),
  in sim and wall time.
"""
import re, sys
from collections import defaultdict
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BEH = ('spin', 'backup', 'wait', 'drive_on_heading')
for arg in sys.argv[1:]:
    label, run = arg.split('=', 1)
    meta = open(run + '/meta.txt').read()
    ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1)); me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
    log = open(run + '/launch.log', errors='ignore').read().splitlines()
    def stamped(pat):
        return [(float(m.group(1)), m.groups()[1:]) for l in log for m in [re.search(r'\[(\d{10}\.\d+)\].*' + pat, l)] if m]
    acks = [(t, g[0]) for t, g in stamped(r'acknowledge goal request for (\w+)') if ms <= t <= me]
    svc = [t for t, _ in stamped(r'(timed out while executing service call)') if ms <= t <= me]
    halt = [t for t, _ in stamped(r'(Failed to get result for \w+ in node halt|Failed to cancel action server)') if ms <= t <= me]
    pfail = [t for t, _ in stamped(r'(failed to create plan)') if ms <= t <= me]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    first = defaultdict(list); seen = set(); execs = defaultdict(list); clk = []; rates = defaultdict(list)
    while r.has_next():
        t, d, ts = r.read_next(); w = ts * 1e-9
        if t == '/clock':
            m = deserialize_message(d, ty[t]); clk.append((w, m.clock.sec + m.clock.nanosec * 1e-9))
        elif t in ('/cmd_vel_nav', '/cmd_vel', '/plan'):
            rates[t].append(w)
        elif t.endswith('/_action/status'):
            a = t.split('/')[1]
            st = deserialize_message(d, ty[t]).status_list
            for s in st:
                g = bytes(s.goal_info.goal_id.uuid).hex()
                if (a, g) not in seen:
                    seen.add((a, g)); first[a].append(w)
            if a in BEH:
                execs[a].append((w, any(s.status == 2 for s in st)))
    clk = np.array(clk); sim = lambda x: np.interp(x, clk[:, 0], clk[:, 1])
    late = sum(1 for w, a in acks if any(w <= f <= w + 1.0 for f in first.get(a, [])))
    # overlapping behaviours on a 20 ms grid
    grid = np.arange(ms, me, 0.02); active = np.zeros((len(BEH), len(grid)), bool)
    for i, a in enumerate(BEH):
        ev = execs.get(a, [])
        if ev:
            tw = np.array([e[0] for e in ev]); on = np.array([e[1] for e in ev])
            idx = np.searchsorted(tw, grid) - 1
            active[i] = np.where(idx >= 0, on[np.clip(idx, 0, None)], False)
    overlap = float(np.sum(active.sum(0) >= 2) * 0.02)
    mins = (me - ms) / 60
    by = defaultdict(int)
    for _, a in acks:
        by[a] += 1
    print(f'{label}: mission {me-ms:.0f} s wall; RTF {(sim(me)-sim(ms))/(me-ms):.3f}')
    print(f'  ack timeouts {len(acks)} ({len(acks)/mins:.2f}/wall min) by action {dict(by)}; service-call timeouts {len(svc)}; '
          f'halt cancel/result errors {len(halt)}; "failed to create plan" {len(pfail)}')
    print(f'  late-accepted (orphaned) goals after an ack timeout: {late}; time with >= 2 behaviours executing at once: {overlap:.1f} s')
    for tpc in ('/cmd_vel_nav', '/cmd_vel', '/plan'):
        x = np.array([v for v in rates[tpc] if ms <= v <= me])
        if len(x) > 2:
            print(f'  {tpc:13s} {len(x)/(sim(x[-1])-sim(x[0])):5.2f} Hz sim, {len(x)/(x[-1]-x[0]):5.2f} Hz wall (mission window)')
