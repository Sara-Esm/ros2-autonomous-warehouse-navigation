#!/usr/bin/env python3
"""Phase 6 verification and shutdown metrics for a recorded mission.

Usage: phase6_metrics.py <label>=<run_dir> [...]
- Speed at NavigateToPose acceptance: truth /odom at bag receipt of each SUCCEEDED status.
- Settling: mission log 'SETTLED: <name> X s after goal success' (Phase 6 node only).
- Marker evidence: 'Correct marker detected ... (image age N ms, captured X s after settling ...)'.
- Verification latency: 'ARRIVED' -> 'Correct marker detected' (wall s, mission log).
- Overhead: 'ARRIVED' -> 'VISION VERIFIED' (wall s); includes settling for the Phase 6 node.
- Final check and summary lines; shutdown from meta.txt and launch.log exit lines.
Runs without Phase 6 log lines report n/a for those fields.
"""
import re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

for arg in sys.argv[1:]:
    lab, run = arg.split('=', 1)
    log = open(run + '/mission.log', errors='ignore').read().splitlines()
    t = lambda l: float(re.search(r'\[(\d{10}\.\d+)\]', l).group(1))
    arrived = [t(l) for l in log if '] ARRIVED:' in l or ']: ARRIVED:' in l]
    correct = [(t(l), l) for l in log if 'Correct marker detected' in l]
    verified = [t(l) for l in log if 'VISION VERIFIED' in l]
    settled = [float(m.group(1)) for l in log for m in [re.search(r'SETTLED: .* ([\d.]+) s after goal success', l)] if m]
    ages = [float(m.group(1)) for _, l in correct for m in [re.search(r'image age (\d+) ms', l)] if m]
    after = [float(m.group(1)) for _, l in correct for m in [re.search(r'captured ([\d.]+) s after settling', l)] if m]
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {x.name: get_message(x.type) for x in r.get_all_topics_and_types()}; od, succ, seen = [], [], set()
    while r.has_next():
        tp, d, ts = r.read_next()
        if tp == '/odom':
            m = deserialize_message(d, ty[tp]); od.append((ts * 1e-9, abs(m.twist.twist.linear.x), abs(m.twist.twist.angular.z)))
        elif tp == '/navigate_to_pose/_action/status':
            for s in deserialize_message(d, ty[tp]).status_list:
                g = bytes(s.goal_info.goal_id.uuid).hex()
                if s.status == 4 and g not in seen: seen.add(g); succ.append(ts * 1e-9)
    od = np.array(od)
    acc = [od[min(np.searchsorted(od[:, 0], w), len(od) - 1), 1:] for w in sorted(succ)]
    print(f'{lab}:')
    print('  speed at goal acceptance (|v| mm/s, |w| rad/s): ' + ', '.join(f'{a[0]*1e3:.0f}/{a[1]:.3f}' for a in acc))
    print('  settling after goal success (sim s): ' + (', '.join(f'{x:.2f}' for x in settled) or 'n/a (no settling stage)'))
    print('  marker evidence image age at verification (ms sim): ' + (', '.join(f'{x:.0f}' for x in ages) or 'n/a'))
    print('  evidence captured after settling (sim s): ' + (', '.join(f'{x:.2f}' for x in after) or 'n/a'))
    lat = [c - a for a, (c, _) in zip(arrived, correct)]
    ovh = [v - a for a, v in zip(arrived, verified)]
    print('  arrival -> correct marker (wall s): ' + ', '.join(f'{x:.2f}' for x in lat) + f' | mean {np.mean(lat):.2f}')
    print('  arrival -> VISION VERIFIED (wall s): ' + ', '.join(f'{x:.2f}' for x in ovh) + f' | mean {np.mean(ovh):.2f}')
    for key in ('FINAL CHECK', 'WAREHOUSE MISSION', 'Successful tasks', 'Failed tasks', 'MISSION HALTED', 'SETTLE FAILED', 'VERIFICATION FAILED'):
        for l in log:
            if key in l:
                print('  log: ' + l.split(']: ', 1)[-1].strip())
    meta = open(run + '/meta.txt').read()
    for key in ('mission_exit_code', 'shutdown_seconds', 'runner TERM needed', 'SIGKILL survivors', 'process_group_empty_after_teardown'):
        m = re.search(rf'{key}: ?(.*)', meta)
        if m: print(f'  meta {key}: {m.group(1).strip()}')
    launch = re.sub(r'\x1b\[[0-9;]*m', '', open(run + '/launch.log', errors='ignore').read())
    for m in re.finditer(r'\[(INFO|ERROR)\] \[([\w.-]+)\]: process has (finished cleanly|died) \[pid \d+(?:, exit code (-?\d+))?', launch):
        print(f'  exit: {m.group(2):34s} {m.group(3)}{" code " + m.group(4) if m.group(4) else ""}')
    for m in re.finditer(r'\[ERROR\] \[([\w.-]+)\]: process\[[\w.-]+\] failed to terminate \'([\d.]+)\' seconds after receiving \'(\w+)\'', launch):
        print(f'  escalation: {m.group(1)} did not exit {m.group(2)} s after {m.group(3)}')
