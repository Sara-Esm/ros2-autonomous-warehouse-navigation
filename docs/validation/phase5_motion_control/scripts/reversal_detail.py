#!/usr/bin/env python3
"""Characterize cruise angular reversals: amplitude (peak-to-peak omega), linear speed, heading-rate context,
and whether the turn rate is saturated against the DWB velocity window. Usage: reversal_detail.py <run_dir>"""
import re, sys
import numpy as np
sys.path.insert(0, 'diagnostics_phase5'); import motion_metrics as mm  # noqa
run = sys.argv[1]
sim, nav, cmd, od = mm.load(run)
ev = [(float(m.group(1)), m.group(2)) for l in open(run + '/mission.log')
      for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)', l)] if m]
legs = [(float(sim(a[0])), float(sim(b[0]))) for a, b in zip(ev, ev[1:]) if a[1] == 'Navigating to' and b[1] == 'ARRIVED']
amp, spd, mean_w, gap = [], [], [], []
for t0, t1 in legs:
    c = nav[(nav[:, 0] >= t0) & (nav[:, 0] <= t1)]
    t, v, w = c[:, 0], c[:, 1], c[:, 2]
    s = np.where(w >= 0.1, 1, np.where(w <= -0.1, -1, 0)); idx = np.nonzero(s)[0]
    for a, b in zip(idx, idx[1:]):
        if s[a] != s[b] and t[b] - t[a] < 1.0 and abs(v[b]) >= 0.10:
            win = (t >= t[a] - 1.0) & (t <= t[b] + 1.0)
            amp.append(w[win].max() - w[win].min()); spd.append(abs(v[b])); gap.append(t[b] - t[a])
            mean_w.append(abs(np.mean(w[(t >= t[b] - 2) & (t <= t[b] + 2)])))
amp, spd, gap, mean_w = map(np.array, (amp, spd, gap, mean_w))
print(f'{run}: {len(amp)} cruise reversals (DWB output)')
print(f'  peak-to-peak omega within +-1 s: median {np.median(amp):.2f}, p90 {np.percentile(amp,90):.2f}, max {amp.max():.2f} rad/s')
print(f'  time between opposite-sign samples: median {np.median(gap)*1e3:.0f} ms sim (controller period ~93 ms)')
print(f'  linear speed at reversal: median {np.median(spd):.3f} m/s (max_vel_x 0.22)')
print(f'  |mean omega| over +-2 s around reversal: median {np.median(mean_w):.3f} rad/s  (small = weave on a straight, large = within a turn)')
# heading weave on straights: truth heading deviation from 1 s moving mean while |v|>=0.15 and |mean w|<0.05
tg = np.arange(od[0, 0], od[-1, 0], 0.02); th = np.unwrap(np.interp(tg, od[:, 0], np.unwrap(od[:, 3]))); vv = np.interp(tg, od[:, 0], od[:, 4])
k = 50; m = np.convolve(th, np.ones(k) / k, mode='same'); dev = np.degrees(th - m)
straight = (np.abs(vv) >= 0.15) & (np.abs(np.gradient(m, tg)) < 0.05)
print(f'  truth heading deviation from its 1 s mean on straights: RMS {np.sqrt(np.mean(dev[straight]**2)):.2f} deg, p99 {np.percentile(np.abs(dev[straight]),99):.2f} deg ({straight.mean()*100:.0f}% of samples)')
