#!/usr/bin/env python3
"""Phase of BT 'acknowledge goal request' timeouts relative to /clock arrival.

Usage: ack_timeout_phase.py <run_dir>
If BtActionNode measured elapsed time on the quantized sim clock (0.1 s steps), timeouts
could only be detected right after a /clock message; a uniform phase indicates wall time.
"""
import re, sys, numpy as np, rosbag2_py
from scipy.stats import kstest
run = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
cw = []
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/clock':
        cw.append(ts * 1e-9)
cw = np.array(cw)
warns = [float(m.group(1)) for l in open(run + '/launch.log') for m in
         [re.search(r'\[(\d{10}\.\d+)\].*acknowledge goal request for (\w+)', l)] if m]
kinds = [m.group(1) for l in open(run + '/launch.log') for m in [re.search(r'acknowledge goal request for (\w+)', l)] if m]
ph = []
for w in warns:
    i = np.searchsorted(cw, w) - 1
    ph.append((w - cw[i]) / (cw[i + 1] - cw[i]))
ph = np.array(ph)
print('timeouts by action:', {k: kinds.count(k) for k in set(kinds)})
print('n =', len(ph), '; phase as fraction of the /clock inter-arrival period:', np.round(np.sort(ph), 2).tolist())
print(f'fraction in first 15% of period: {np.mean(ph < 0.15):.2f} (uniform: 0.15; sim-clock detection: ~1.0)')
print(f'KS test vs uniform: p = {kstest(ph, "uniform").pvalue:.3f}')
