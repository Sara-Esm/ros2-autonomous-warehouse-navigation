#!/usr/bin/env python3
"""Wall- vs sim-pacing of Nav2 loops in recorded missions.

Usage: loop_timing.py <label>=<run_dir> [...]
For each topic: rate in wall time (bag receive) and sim time (/clock-mapped), wall/sim ratio vs RTF.
Global costmap: wall interval between publications and sim (header) interval, compared with the
prediction from costmap_2d_ros.cpp: publish when sim time since last publish exceeds 1/publish_frequency,
checked once per WallRate(update_frequency) iteration -> sim interval = n * T_wall_iter * RTF with n the
smallest integer making it exceed 1 s.
"""
import math, re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for arg in sys.argv[1:]:
    label, run = arg.split('=', 1)
    meta = open(run + '/meta.txt').read()
    ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1)); me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    clk, w = [], {k: [] for k in ('/cmd_vel_nav', '/cmd_vel', '/plan', '/global_costmap/costmap')}; cmh = []
    while r.has_next():
        t, d, ts = r.read_next(); tw = ts * 1e-9
        if not ms <= tw <= me: continue
        if t == '/clock':
            m = deserialize_message(d, ty[t]); clk.append((tw, m.clock.sec + m.clock.nanosec * 1e-9))
        elif t in w:
            w[t].append(tw)
            if t == '/global_costmap/costmap':
                m = deserialize_message(d, ty[t]); cmh.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    clk = np.array(clk); sim = lambda x: np.interp(x, clk[:, 0], clk[:, 1])
    rtf = (sim(me) - sim(ms)) / (me - ms)
    print(f'{label}: RTF {rtf:.3f}')
    for t, x in w.items():
        x = np.array(x); fw = (len(x) - 1) / (x[-1] - x[0]); fs = (len(x) - 1) / (sim(x[-1]) - sim(x[0]))
        print(f'   {t:24s} {fw:6.2f} Hz wall {fs:6.2f} Hz sim   wall/sim ratio {fw/fs:.3f} (RTF {rtf:.3f})')
    x = np.array(w['/global_costmap/costmap']); iw = np.diff(x); ih = np.diff(cmh)
    Tw = np.median(iw) / 2
    n = math.floor(1.0 / (Tw * rtf)) + 1
    print(f'   global costmap: wall interval median {np.median(iw):.2f} s (p5 {np.percentile(iw,5):.2f}, p95 {np.percentile(iw,95):.2f}); '
          f'sim header interval median {np.median(ih):.2f} s; predicted sim interval n*T*RTF = {n}*{Tw:.2f}*{rtf:.3f} = {n*Tw*rtf:.2f} s')
