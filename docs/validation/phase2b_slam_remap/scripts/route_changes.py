#!/usr/bin/env python3
"""Count route changes (> 0.5 m one-sided Hausdorff between consecutive /plan messages) per leg.

Usage: route_changes.py <label>=<run_dir> [...]
"""
import re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for arg in sys.argv[1:]:
    label, run = arg.split('=', 1)
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in rd.get_all_topics_and_types()}; plans = []
    while rd.has_next():
        t, d, ts = rd.read_next()
        if t == '/plan':
            m = deserialize_message(d, ty[t])
            if len(m.poses) > 1:
                plans.append((ts * 1e-9, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
    ev = [(float(m.group(1)), m.group(2), m.group(3)) for l in open(run + '/mission.log')
          for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)(.*)', l)] if m]
    legs = [(n.split(':')[0].strip(), a, b) for (a, k, n), (b, _, _) in zip(ev, ev[1:]) if k == 'Navigating to']
    print(f'{label}:')
    for name, a, b in legs:
        lp = [xy for w, xy in plans if a <= w <= b]; ds = []
        for p, q in zip(lp, lp[1:]):
            sub = q[::max(1, len(q) // 50)]
            ds.append(max(np.min(np.hypot(p[:, 0] - s[0], p[:, 1] - s[1])) for s in sub))
        print(f'  {name:28s} plans {len(lp):3d}, route changes > 0.5 m: {sum(d > 0.5 for d in ds)}, '
              f'largest consecutive-plan change {max(ds):.2f} m')
