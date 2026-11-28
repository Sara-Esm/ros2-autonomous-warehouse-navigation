#!/usr/bin/env python3
"""
Summarize a dwb_window.npz (Loading -> Inventory departure) and judge whether a departure stall occurred.

Usage: dwb_window_summary.py <label>=<run_dir> [...]
Per run: evaluations recorded and their rate; candidate trajectories per cycle and how many were valid;
time until the chosen command first reaches |vtheta| >= 0.3 rad/s or vx >= 0.05 m/s (departure start);
the chosen command's mean |vtheta| over the first 5 s; and the mean best-trajectory critic contribution
(raw score x scale) over that window, which shows which critics dominated the choice.
Stall = no chosen command reaching those thresholds within the first 5 s.
"""
import sys
import numpy as np

for arg in sys.argv[1:]:
    lab, run = arg.split('=', 1)
    try:
        z = np.load(f'{run}/dwb_window.npz')
    except FileNotFoundError:
        print(f'{lab}: no dwb_window.npz'); continue
    r = z['rows']
    if len(r) == 0:
        print(f'{lab}: no evaluations recorded (window start {float(z["window_start"]):.1f})'); continue
    t = r[:, 0] - float(z['window_start'])
    ntraj, nvalid, vx, vth = r[:, 2], r[:, 3], r[:, 5], r[:, 6]
    go = np.nonzero((np.abs(vth) >= 0.3) | (vx >= 0.05))[0]
    t_go = t[go[0]] if len(go) else float('inf')
    first5 = t <= 5.0
    crit = {k[len('critic_'):]: z[k] for k in z.files if k.startswith('critic_')}
    dom = sorted(((np.mean(v[:first5.sum()]) if first5.sum() else np.nan, k) for k, v in crit.items()), reverse=True)
    print(f'{lab}: {len(r)} evaluations in {t[-1]:.1f} s wall ({len(r)/max(t[-1],1e-3):.1f} Hz); '
          f'candidates per cycle {int(np.median(ntraj))}, valid median {int(np.median(nvalid))} (min {int(nvalid.min())}); '
          f'departure command at {t_go:.2f} s wall after the goal; mean |vtheta| first 5 s {np.mean(np.abs(vth[first5])):.2f} rad/s; '
          f'{"STALL" if t_go > 5.0 else "no stall"}')
    print('   best-trajectory critic terms, first 5 s (mean raw x scale): ' + ', '.join(f'{k} {v:.1f}' for v, k in dom))
