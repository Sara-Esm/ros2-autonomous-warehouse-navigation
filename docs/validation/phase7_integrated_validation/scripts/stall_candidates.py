#!/usr/bin/env python3
"""Candidate-trajectory costs at recorded departure cycles. Usage: stall_candidates.py <label>=<run_dir> ...
For each stored full candidate set (first 200 cycles): chosen trajectory, best valid left turn (vtheta >= 0.25),
best valid right turn (vtheta <= -0.25) and best near-still candidate (|vtheta| < 0.05, vx < 0.02).
DWB total: lower is better; negative = invalid. 'n/a' when no valid candidate exists in a category."""
import sys
import numpy as np
for arg in sys.argv[1:]:
    lab, run = arg.split('=', 1)
    z = np.load(f'{run}/dwb_window.npz'); r = z['rows']; t0 = float(z['window_start'])
    print(f'{lab}:')
    for i in range(min(len(r), 200)):
        f = z[f'full_{i:03d}']; ok = f[:, 2] >= 0
        def best(mask):
            m = mask & ok
            return f[m][np.argmin(f[m][:, 2])] if m.any() else None
        ch = best(np.ones(len(f), bool)); L = best(f[:, 1] >= 0.25); R = best(f[:, 1] <= -0.25)
        S = best((np.abs(f[:, 1]) < 0.05) & (f[:, 0] < 0.02))
        fmt = lambda c: 'n/a' if c is None else f'{c[1]:+.2f} rad/s cost {c[2]:.2f}'
        print(f'  t={r[i,0]-t0:5.2f} s  valid {int(ok.sum())}/{len(f)}  chosen vx {ch[0]:+.3f} vth {ch[1]:+.3f} cost {ch[2]:.2f}'
              f' | left {fmt(L)} | right {fmt(R)} | near-still {"n/a" if S is None else f"cost {S[2]:.2f}"}')
