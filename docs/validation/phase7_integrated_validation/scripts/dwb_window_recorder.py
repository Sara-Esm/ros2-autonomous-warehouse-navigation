#!/usr/bin/env python3
"""
Record DWB trajectory evaluations during one mission leg only (Phase 7 departure-stall evidence).

Usage: dwb_window_recorder.py <out_npz> [window_s]
Waits for the mission's rosout line 'Navigating to Inventory Scan Station', then subscribes to
/evaluation (dwb_msgs/LocalPlanEvaluation) for window_s seconds of wall time and unsubscribes.
DWB (Nav2 1.1.18) builds evaluations every cycle by default but publishes them only while a
subscriber exists, so publishing is active only inside this window. Per cycle it stores: receipt
wall time, header stamp, best index, number of trajectories and how many are valid (finite total
>= 0), the best trajectory's (vx, vtheta, total), and per critic the best trajectory's raw score x
scale. The full trajectory set (every candidate's vx, vtheta, total) is kept for the first 200 cycles.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Log
from dwb_msgs.msg import LocalPlanEvaluation

out = sys.argv[1]
window = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
TRIGGER = 'Navigating to Inventory Scan Station'

rclpy.init()
n = Node('dwb_window_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
state = {'start': None, 'sub': None}
rows, critics, full = [], {}, []


def on_eval(m):
    tot = np.array([t.total for t in m.twists], dtype=float)
    valid = int(np.sum(np.isfinite(tot) & (tot >= 0)))
    b = m.twists[m.best_index] if m.twists else None
    rows.append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, len(m.twists), valid,
                 m.best_index, b.traj.velocity.x if b else np.nan, b.traj.velocity.theta if b else np.nan,
                 b.total if b else np.nan))
    if b:
        for s in b.scores:
            critics.setdefault(s.name, []).append(s.raw_score * s.scale)
    if len(full) < 200:
        full.append(np.array([(t.traj.velocity.x, t.traj.velocity.theta, t.total) for t in m.twists]))


def on_log(m):
    if state['start'] is None and m.name == 'warehouse_autonomy' and TRIGGER in m.msg:
        state['start'] = time.time()
        state['sub'] = n.create_subscription(LocalPlanEvaluation, '/evaluation', on_eval, 10)


n.create_subscription(Log, '/rosout', on_log, 50)
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if state['start'] is not None and time.time() - state['start'] > window:
            n.destroy_subscription(state['sub'])
            break
except KeyboardInterrupt:
    pass
r = np.array(rows, dtype=float)
np.savez_compressed(out, rows=r, window_start=state['start'] or np.nan,
                    **{f'critic_{k}': np.array(v) for k, v in critics.items()},
                    **{f'full_{i:03d}': f for i, f in enumerate(full)})
print(f'recorded {len(rows)} evaluations from {state["start"]}', flush=True)
n.destroy_node()
rclpy.try_shutdown()
