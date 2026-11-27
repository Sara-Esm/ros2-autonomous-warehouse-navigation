#!/usr/bin/env python3
"""Do cruise reversals follow new plans? Usage: reversal_vs_replan.py <run_dir> ...
For each cruise reversal (definition as motion_metrics.py, DWB output), time since the latest /plan (sim).
Compared with the same statistic for uniformly random times within the cruise samples (chance level).
Also: lateral shift between consecutive plans at the robot's position (closest-point offset)."""
import re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, 'diagnostics_phase5'); import motion_metrics as mm  # noqa
rng = np.random.default_rng(0)
for run in sys.argv[1:]:
    sim, nav, cmd, od = mm.load(run)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}; plans = []
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/plan':
            m = deserialize_message(d, ty[t]); plans.append((float(sim(ts * 1e-9)), np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
    pt = np.array([p[0] for p in plans])
    ev = [(float(m.group(1)), m.group(2)) for l in open(run + '/mission.log') for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)', l)] if m]
    legs = [(float(sim(a[0])), float(sim(b[0]))) for a, b in zip(ev, ev[1:]) if a[1] == 'Navigating to' and b[1] == 'ARRIVED']
    rev_t, cruise_t = [], []
    for t0, t1 in legs:
        c = nav[(nav[:, 0] >= t0) & (nav[:, 0] <= t1)]; t, v, w = c[:, 0], c[:, 1], c[:, 2]
        cruise_t += list(t[np.abs(v) >= 0.10])
        s = np.where(w >= 0.1, 1, np.where(w <= -0.1, -1, 0)); idx = np.nonzero(s)[0]
        for a, b in zip(idx, idx[1:]):
            if s[a] != s[b] and t[b] - t[a] < 1.0 and abs(v[b]) >= 0.10: rev_t.append(t[a])   # onset of the swing
    def since(x):
        i = np.searchsorted(pt, x) - 1; return np.where(i >= 0, x - pt[np.clip(i, 0, None)], np.nan)
    rs = since(np.array(rev_t)); cs = since(rng.choice(np.array(cruise_t), 5000))
    period = np.median(np.diff(pt))
    print(f'{run}: {len(rev_t)} reversals; plan interval median {period:.2f} s sim')
    for w in (0.3, 0.5):
        print(f'  reversal onset within {w:.1f} s after a new plan: {np.mean(rs < w)*100:.0f}%  (chance {np.mean(cs < w)*100:.0f}%)')
    # plan-to-plan lateral shift near the robot: distance from new plan's closest point to robot vs old plan's
    sh = []
    o = od
    for (ta, pa), (tb, pb) in zip(plans, plans[1:]):
        if len(pa) < 5 or len(pb) < 5: continue
        k = np.searchsorted(o[:, 0], tb); 
        if k >= len(o): continue
        rx, ry = o[k, 1], o[k, 2]
        ahead = pb[np.argmin(np.hypot(pb[:, 0] - pb[0, 0], pb[:, 1] - pb[0, 1]) < 0) :][:12]   # first ~0.6 m of the new plan
        da = [np.min(np.hypot(pa[:, 0] - x, pa[:, 1] - y)) for x, y in ahead]
        sh.append(np.max(da))
    sh = np.array(sh) * 100
    print(f'  new plan vs previous plan over its first ~0.6 m: max offset median {np.median(sh):.1f} cm, p90 {np.percentile(sh,90):.1f} cm, max {sh.max():.1f} cm (n {len(sh)})')
