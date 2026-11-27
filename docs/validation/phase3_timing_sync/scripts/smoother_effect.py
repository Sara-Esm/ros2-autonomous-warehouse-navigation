#!/usr/bin/env python3
"""Velocity-smoother behaviour before/after its clock change: DWB (/cmd_vel_nav) vs smoother (/cmd_vel)
vs truth (/odom) during navigation legs. Lag from cross-correlation on a 10 ms wall grid; message rates."""
import re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for arg in sys.argv[1:]:
    lab, R = arg.split('=', 1)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=R + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}; S = {'/cmd_vel_nav': [], '/cmd_vel': [], '/odom': [], '/clock': []}; iv = {}
    while r.has_next():
        t, d, ts = r.read_next()
        if t in ('/spin/_action/status', '/backup/_action/status', '/wait/_action/status', '/drive_on_heading/_action/status'):
            for st in deserialize_message(d, ty[t]).status_list:
                g = bytes(st.goal_info.goal_id.uuid).hex()
                if st.status == 2: iv.setdefault(g, [ts * 1e-9, None])
                if st.status in (4, 5, 6) and g in iv and iv[g][1] is None: iv[g][1] = ts * 1e-9
        if t in S:
            m = deserialize_message(d, ty[t])
            if t == '/clock': S[t].append((ts * 1e-9, m.clock.sec + m.clock.nanosec * 1e-9))
            else:
                tw = m.twist.twist if t == '/odom' else m; S[t].append((ts * 1e-9, tw.linear.x, tw.angular.z))
    ev = [(float(m.group(1)), m.group(2)) for l in open(R + '/mission.log') for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)', l)] if m]
    legs = [(a, b) for (a, k), (b, _) in zip(ev, ev[1:]) if k == 'Navigating to']
    A = {k: np.array(v) for k, v in S.items()}
    grid = np.concatenate([np.arange(a, b, 0.01) for a, b in legs])
    keep = np.ones(len(grid), bool)
    for a, b in iv.values():
        keep &= ~((grid >= a - 1.0) & (grid <= (b if b else a) + 1.0))
    excluded = 100 * (1 - keep.mean()); grid = grid[keep]
    f = lambda k, c: np.interp(grid, A[k][:, 0], A[k][:, c])
    def lag(x, y):
        best = (-2, 0)
        for k in range(0, 50):
            c = np.corrcoef(x[:len(x) - k], y[k:])[0, 1]
            if c > best[0]: best = (c, k * 10)
        return best
    out = [f'{lab}: (recovery periods +-1 s excluded: {excluded:.1f}% of leg time)']
    for c, nm in ((1, 'v'), (2, 'w')):
        n, s, o = f('/cmd_vel_nav', c), f('/cmd_vel', c), f('/odom', c)
        cs, ls = lag(n, s); co, lo = lag(s, o)
        k = ls // 10
        rms = np.sqrt(np.mean((s[k:] - n[:len(n) - k]) ** 2))
        out.append(f'  {nm}: DWB->smoother lag {ls} ms (corr {cs:.3f}), RMS smoother-vs-DWB after lag {rms:.4f}; smoother->truth lag {lo} ms (corr {co:.3f})')
    clk = A['/clock']; sim = lambda x: np.interp(x, clk[:, 0], clk[:, 1])
    for tpc in ('/cmd_vel_nav', '/cmd_vel'):
        x = np.array([v for v in A[tpc][:, 0] if any(a <= v <= b for a, b in legs)])
        out.append(f'  {tpc}: {len(x)/(sim(x[-1])-sim(x[0])):.2f} Hz sim, {len(x)/(x[-1]-x[0]):.2f} Hz wall during legs')
    print('\n'.join(out))
