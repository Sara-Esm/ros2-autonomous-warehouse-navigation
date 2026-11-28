#!/usr/bin/env python3
"""Angular-acceleration estimates from truth /odom under several differentiation methods (navigation legs only).

Usage: angular_accel_methods.py <label>=<run_dir> ...
Methods on the raw 34 ms-stamped samples (header stamps, sim time):
  per_sample : |dw|/dt between consecutive samples
  orig_0.1s  : the Phase 5 motion_metrics method (interpolate to 10 ms, 0.1 s box filter, difference)
  ls_0.1s    : least-squares slope of w over a centred 0.1 s window of raw samples (about 3-4 samples)
  ls_0.2s    : same over 0.2 s (about 6-7 samples)
Reported: p95 and max over the navigation legs ('Navigating to' -> 'ARRIVED').
"""
import re, sys
import numpy as np
sys.path.insert(0, 'diagnostics_phase5'); import motion_metrics as mm  # noqa


def ls_slope(t, w, half):
    out = np.full(len(t), np.nan)
    j0 = j1 = 0
    for i in range(len(t)):
        while t[j0] < t[i] - half: j0 += 1
        while j1 + 1 < len(t) and t[j1 + 1] <= t[i] + half: j1 += 1
        tt, ww = t[j0:j1 + 1], w[j0:j1 + 1]
        if len(tt) >= 3:
            out[i] = np.polyfit(tt - t[i], ww, 1)[0]
    return np.abs(out)


for arg in sys.argv[1:]:
    lab, run = arg.split('=', 1)
    sim, nav, cmd, od = mm.load(run)
    ev = [(float(m.group(1)), m.group(2)) for l in open(run + '/mission.log')
          for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)', l)] if m]
    legs = [(float(sim(a[0])), float(sim(b[0]))) for a, b in zip(ev, ev[1:]) if a[1] == 'Navigating to' and b[1] == 'ARRIVED']
    inleg = lambda t: np.any([(t >= a) & (t <= b) for a, b in legs], axis=0)
    t, w = od[:, 0], od[:, 5]
    res = {}
    ps = np.abs(np.diff(w)) / np.diff(t); res['per_sample'] = ps[inleg(t[1:])]
    tg = np.arange(t[0], t[-1], 0.01); ws = np.convolve(np.interp(tg, t, w), np.ones(10) / 10, mode='same')
    og = np.abs(np.diff(ws)) / 0.01; res['orig_0.1s'] = og[inleg(tg[1:])]
    res['ls_0.1s'] = ls_slope(t, w, 0.05)[inleg(t)]; res['ls_0.2s'] = ls_slope(t, w, 0.10)[inleg(t)]
    print(f'{lab}: ' + ' | '.join(f'{k} p95 {np.nanpercentile(v,95):.2f} max {np.nanmax(v):.1f}' for k, v in res.items()) + '  (rad/s^2)')
