#!/usr/bin/env python3
"""Offline controller-dynamics analysis of the recorded diagnostic mission.

Usage: controller_dynamics.py <run_dir>

Inputs (recorded; no simulation needed): /odom (Gazebo world-truth twist, sim stamps),
/cmd_vel_nav (DWB output), /cmd_vel (velocity_smoother output), /clock.
Twist commands carry no timestamp: their sim time is reconstructed from bag receipt
(wall) time via /clock interpolation (uncertainty discussed in the output).

Outputs in <run_dir>/analysis/controller_dynamics/: results.txt, windows.csv, *.png
"""
import math
import re
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RUN = Path(sys.argv[1])
OUT = RUN / 'analysis' / 'controller_dynamics'
OUT.mkdir(parents=True, exist_ok=True)
B = 0.160                    # wheel separation (robot SDF)
WHEEL_LIMIT = 1.0            # Gazebo diff-drive max_wheel_acceleration (SDF)
DWB = dict(acc_x=2.5, dec_x=2.5, acc_th=3.2, dec_th=3.2, period=0.1, vmax=0.22, wmax=1.0)
SMOOTHER = dict(acc_x=2.5, acc_th=3.2, hz=20.0)   # Humble defaults, wall clock (use_sim_time false)
LINES = []


def P(s=''):
    LINES.append(s)
    print(s)


# ------------------------------------------------------------------ load
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(RUN / 'bag'), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
clk, od, cn, cv = [], [], [], []
while r.has_next():
    t, d, ts = r.read_next()
    w = ts * 1e-9
    if t == '/clock':
        m = deserialize_message(d, ty[t]); clk.append((w, m.clock.sec + m.clock.nanosec * 1e-9))
    elif t == '/odom':
        m = deserialize_message(d, ty[t])
        od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, w, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif t == '/cmd_vel_nav':
        m = deserialize_message(d, ty[t]); cn.append((w, m.linear.x, m.angular.z))
    elif t == '/cmd_vel':
        m = deserialize_message(d, ty[t]); cv.append((w, m.linear.x, m.angular.z))
clk, od, cn, cv = map(np.array, (clk, od, cn, cv))
# keep only the part of the run with a live simulation (before gzserver died)
t_end = od[-1, 0]
sim_of = lambda w: np.interp(w, clk[:, 0], clk[:, 1])
cn = np.c_[sim_of(cn[:, 0]), cn]; cv = np.c_[sim_of(cv[:, 0]), cv]
cn = cn[cn[:, 0] <= t_end]; cv = cv[cv[:, 0] <= t_end]
od = od[np.r_[True, np.diff(od[:, 0]) > 0]]

# mission window (from mission log wall times) -> sim
ev = [(float(m.group(1)), m.group(2)) for l in open(RUN / 'mission.log') for m in
      [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED|MISSION HALTED)', l)] if m]
legs = []
for (w0, k0), (w1, k1) in zip(ev, ev[1:]):
    if k0 == 'Navigating to':
        legs.append((float(sim_of(w0)), min(float(sim_of(w1)), t_end)))

# ------------------------------------------------------------------ sampling facts
dt_od = np.diff(od[:, 0])
P('# Controller dynamics (offline, recorded data only)')
P(f'odom samples {len(od)}; sim-stamp interval median {np.median(dt_od)*1e3:.1f} ms (p5 {np.percentile(dt_od,5)*1e3:.1f}, p95 {np.percentile(dt_od,95)*1e3:.1f}, max {dt_od.max()*1e3:.0f})')
dcw = np.diff(clk[:, 0])
P(f'/clock: 0.100 s sim per message, received every {np.median(dcw)*1e3:.0f} ms wall (p95 {np.percentile(dcw,95)*1e3:.0f}); '
  f'command sim times are interpolated between /clock messages -> timing uncertainty of order +-0.05 s sim')
P(f'cmd_vel_nav {len(cn)} msgs, median sim interval {np.median(np.diff(cn[:,0]))*1e3:.0f} ms; cmd_vel {len(cv)} msgs, median sim interval {np.median(np.diff(cv[:,0]))*1e3:.0f} ms')

# ------------------------------------------------------------------ uniform grid + filtering
FS = 30.0
tg = np.arange(od[0, 0], t_end, 1 / FS)
v = np.interp(tg, od[:, 0], od[:, 2]); w = np.interp(tg, od[:, 0], od[:, 3])


def savgol_deriv(x, win=9, poly=2, dt=1 / FS):
    """Savitzky-Golay first derivative (window 9 samples = 0.30 s sim)."""
    half = win // 2
    A = np.vander(np.arange(-half, half + 1), poly + 1, increasing=True)
    coef = np.linalg.pinv(A)[1] / dt
    xp = np.r_[np.full(half, x[0]), x, np.full(half, x[-1])]
    return np.convolve(xp, coef[::-1], mode='valid')


a_lin = savgol_deriv(v); a_ang = savgol_deriv(w)
vl, vr = v - w * B / 2, v + w * B / 2
al, ar = savgol_deriv(vl), savgol_deriv(vr)
a_wheel = np.maximum(np.abs(al), np.abs(ar))
moving = np.zeros(len(tg), bool)
for a, b in legs:
    moving |= (tg >= a) & (tg <= b)

P('\n## 1. Achieved acceleration (Gazebo truth, Savitzky-Golay derivative, 0.30 s window, during navigation legs)')
for name, x, lim in [('body linear a [m/s^2]', a_lin, f'DWB {DWB["acc_x"]}, smoother {SMOOTHER["acc_x"]}'),
                     ('body angular alpha [rad/s^2]', a_ang, f'DWB {DWB["acc_th"]}, smoother {SMOOTHER["acc_th"]}'),
                     ('wheel rim accel max(|aL|,|aR|) [m/s^2]', a_wheel, f'Gazebo plugin {WHEEL_LIMIT}')]:
    xm = np.abs(x[moving])
    P(f'  {name:42s} p50 {np.percentile(xm,50):5.2f}  p95 {np.percentile(xm,95):5.2f}  p99 {np.percentile(xm,99):5.2f}  max {xm.max():5.2f}   (limits: {lim})')
# Sensitivity to filter choice
for win in (5, 15):
    aw = np.maximum(np.abs(savgol_deriv(vl, win)), np.abs(savgol_deriv(vr, win)))[moving]
    P(f'  sensitivity: SG window {win} ({win/FS:.2f} s): wheel accel p99 {np.percentile(aw,99):.2f} max {aw.max():.2f}')
# Largest step responses: speed-up from standstill
P('  Speed-ups from rest (true v < 0.01 then > 0.15 within 1.5 s): rise time 0->0.9*v_final and mean accel')
starts = np.nonzero((v[:-1] < 0.01) & (v[1:] >= 0.01))[0]
for i in starts:
    j = i + int(1.5 * FS)
    if j < len(v) and v[j] > 0.15:
        vf = v[j]; k = i + np.argmax(v[i:j] >= 0.9 * vf)
        P(f'    t={tg[i]:7.1f} s sim: 0 -> {vf:.3f} m/s; 90% in {(k-i)/FS:.2f} s -> mean {0.9*vf/max((k-i)/FS,1e-3):.2f} m/s^2 (body)')

# ------------------------------------------------------------------ commanded vs achieved
cnv = np.interp(tg, cn[:, 0], cn[:, 2]); cnw = np.interp(tg, cn[:, 0], cn[:, 3])
cvv = np.interp(tg, cv[:, 0], cv[:, 2]); cvw = np.interp(tg, cv[:, 0], cv[:, 3])


def lag(x, y, maxlag=1.0):
    """Lag (s) maximizing correlation of y(t) with x(t - lag); positive -> y follows x."""
    x = x - x.mean(); y = y - y.mean(); best = (-2, 0)
    for k in range(0, int(maxlag * FS) + 1):
        c = np.corrcoef(x[:len(x) - k], y[k:])[0, 1]
        if c > best[0]:
            best = (c, k / FS)
    return best


P('\n## 2. Command chain: DWB -> velocity_smoother -> Gazebo (lag from cross-correlation, moving segments)')
mm = moving
for name, x, y in [('v: DWB -> smoother', cnv[mm], cvv[mm]), ('v: smoother -> truth', cvv[mm], v[mm]), ('v: DWB -> truth', cnv[mm], v[mm]),
                   ('w: DWB -> smoother', cnw[mm], cvw[mm]), ('w: smoother -> truth', cvw[mm], w[mm]), ('w: DWB -> truth', cnw[mm], w[mm])]:
    c, L = lag(x, y)
    P(f'  {name:24s} lag {L*1e3:5.0f} ms (corr {c:.3f})')
_, Lv = lag(cnv[mm], v[mm]); _, Lw = lag(cnw[mm], w[mm])
kv, kw = int(round(Lv * FS)), int(round(Lw * FS))
ev_ = (v[kv:] - cnv[:len(cnv) - kv])[mm[kv:]]; ew_ = (w[kw:] - cnw[:len(cnw) - kw])[mm[kw:]]
P(f'  tracking error after removing lag: |v_true - v_DWB| p50 {np.percentile(np.abs(ev_),50):.3f} p95 {np.percentile(np.abs(ev_),95):.3f} m/s; '
  f'|w_true - w_DWB| p50 {np.percentile(np.abs(ew_),50):.3f} p95 {np.percentile(np.abs(ew_),95):.3f} rad/s')

# ------------------------------------------------------------------ feasibility of each DWB command
P('\n## 3. Feasibility of DWB commands under the wheel constraint')
P('  DWB assumes the commanded (v,w) is reachable within its sim_period (1/controller_frequency = 0.1 s)')
P(f'  from the current velocity, with acc_lim_x {DWB["acc_x"]} and acc_lim_theta {DWB["acc_th"]}: a window of +-{DWB["acc_x"]*DWB["period"]:.2f} m/s and +-{DWB["acc_th"]*DWB["period"]:.2f} rad/s per cycle.')
P(f'  Under the wheel limit ({WHEEL_LIMIT} m/s^2 at each rim) the reachable change in 0.1 s satisfies |dv| + |dw|*b/2 <= {WHEEL_LIMIT*DWB["period"]:.2f} m/s,')
P(f'  i.e. pure linear <= {WHEEL_LIMIT*DWB["period"]:.2f} m/s, pure angular <= {WHEEL_LIMIT*DWB["period"]/(B/2):.2f} rad/s.')
# demand: command vs measured velocity at command time
dem = []
for t_c, w_c, vc, wc in cn[:, :4]:
    if not any(a <= t_c <= b for a, b in legs):
        continue
    va = np.interp(t_c, od[:, 0], od[:, 2]); wa = np.interp(t_c, od[:, 0], od[:, 3])
    dv, dw = vc - va, wc - wa
    need = (abs(dv) + abs(dw) * B / 2) / DWB['period']        # worst-wheel accel needed to reach cmd in 0.1 s
    dem.append((t_c, dv, dw, need))
dem = np.array(dem)
P(f'  DWB commands during legs: {len(dem)}')
for th in (0.5, 1.0, 2.0):
    P(f'    required worst-wheel accel to reach command within 0.1 s > {th:.1f} m/s^2: {100*np.mean(dem[:,3]>th):5.1f}% of commands')
lin_only = np.abs(dem[:, 1]) / DWB['period']; ang_part = np.abs(dem[:, 2]) * B / 2 / DWB['period']
P(f'    of the infeasible (>1.0) commands, dominated by the linear term: {100*np.mean(lin_only[dem[:,3]>1] > ang_part[dem[:,3]>1]):.0f}%')
P(f'  DWB step size between consecutive commands: |dv| p95 {np.percentile(np.abs(np.diff(cn[:,2])),95):.3f} max {np.abs(np.diff(cn[:,2])).max():.3f} m/s; '
  f'|dw| p95 {np.percentile(np.abs(np.diff(cn[:,3])),95):.3f} max {np.abs(np.diff(cn[:,3])).max():.3f} rad/s')
# smoother effect
d_s = np.abs(np.diff(cv[:, 2])) / np.maximum(np.diff(cv[:, 0]), 1e-3)
P(f'  smoother output linear rate-of-change (sim time): p95 {np.percentile(d_s,95):.2f} m/s^2 (its limit is 2.5 m/s^2 in WALL time = {2.5/0.66:.1f} m/s^2 in sim time at RTF 0.66)')

# ------------------------------------------------------------------ association with steering oscillation
P('\n## 4. Association between actuator saturation and steering oscillation (2 s sim windows, |v| > 0.1 m/s)')
WIN = 2.0
rows = []
for a, b in legs:
    for t0 in np.arange(a, b - WIN, WIN):
        s = (tg >= t0) & (tg < t0 + WIN)
        if np.mean(np.abs(v[s]) > 0.1) < 0.8:
            continue
        wt = w[s]; wc = cnw[s]
        hp = wt - np.convolve(wt, np.ones(15) / 15, mode='same')     # high-pass (remove < ~2 Hz trend)
        sig = np.sign(wt[np.abs(wt) > 0.05]); rev = int(np.sum(sig[1:] != sig[:-1])) if len(sig) > 1 else 0
        sigc = np.sign(wc[np.abs(wc) > 0.05]); revc = int(np.sum(sigc[1:] != sigc[:-1])) if len(sigc) > 1 else 0
        dsel = (dem[:, 0] >= t0) & (dem[:, 0] < t0 + WIN)
        sat = np.mean(dem[dsel, 3] > WHEEL_LIMIT) if dsel.sum() else np.nan
        wheel_sat = np.mean(a_wheel[s] > 0.8 * WHEEL_LIMIT)
        rows.append((t0, np.std(hp), rev, revc, sat, wheel_sat, np.std(np.abs(cnv[s] - v[s]))))
rows = np.array(rows)
np.savetxt(OUT / 'windows.csv', rows, delimiter=',', fmt='%.4f',
           header='t0_sim,w_highpass_std,true_w_reversals,cmd_w_reversals,frac_cmd_infeasible,frac_wheel_accel_gt_0.8limit,std_v_error')


def spearman(x, y):
    ok = np.isfinite(x) & np.isfinite(y)
    rx = np.argsort(np.argsort(x[ok])); ry = np.argsort(np.argsort(y[ok]))
    return np.corrcoef(rx, ry)[0, 1], ok.sum()


P(f'  windows: {len(rows)}')
for nm, i in [('infeasible-command fraction', 4), ('wheel-accel near-limit fraction', 5)]:
    rho, n = spearman(rows[:, i], rows[:, 1]); rho2, _ = spearman(rows[:, i], rows[:, 2])
    P(f'  Spearman({nm}, high-pass w std) = {rho:+.2f}; vs true-w reversals = {rho2:+.2f} (n={n})')
hi = rows[:, 4] > np.nanmedian(rows[:, 4]); lo = ~hi
P(f'  high-pass w std: windows with above-median infeasible fraction {np.mean(rows[hi,1]):.3f} vs below {np.mean(rows[lo,1]):.3f} rad/s')
rho, n = spearman(rows[:, 3], rows[:, 2])
P(f'  Spearman(DWB command w reversals, true w reversals) = {rho:+.2f}: the oscillation is already present in the DWB command stream')
P(f'  mean reversals per 2 s window: DWB command {rows[:,3].mean():.1f}, true motion {rows[:,2].mean():.1f}')

# where does high-frequency yaw-rate content originate?
def hp(x): return x - np.convolve(x, np.ones(15) / 15, mode='same')
def band_std(x, lo, hi):
    X = np.fft.rfft((x - x.mean()) * np.hanning(len(x))); f = np.fft.rfftfreq(len(x), 1 / FS)
    return np.sqrt(np.sum(np.abs(X[(f >= lo) & (f < hi)]) ** 2) / np.sum(np.abs(X) ** 2)) * x.std()
P('\n## 5. Origin of high-frequency yaw-rate content (moving segments, per leg then pooled)')
for nm, x in [('DWB /cmd_vel_nav w', cnw), ('smoother /cmd_vel w', cvw), ('truth /odom w', w)]:
    segs_ = [x[(tg >= a) & (tg <= b) & moving] for a, b in legs]
    hps = np.concatenate([hp(sg) for sg in segs_ if len(sg) > 30])
    b1 = np.mean([band_std(sg, 0.3, 2.0) for sg in segs_ if len(sg) > 64])
    b2 = np.mean([band_std(sg, 2.0, 5.0) for sg in segs_ if len(sg) > 64])
    b3 = np.mean([band_std(sg, 5.0, 15.0) for sg in segs_ if len(sg) > 64])
    P(f'  {nm:22s} high-pass std {hps.std():.3f} rad/s | band RMS 0.3-2 Hz {b1:.3f}, 2-5 Hz {b2:.3f}, 5-15 Hz {b3:.3f}')
P('  (DWB publishes at ~15 Hz sim: its content above ~7 Hz is not observable; interpolation smooths its steps)')

(OUT / 'results.txt').write_text('\n'.join(LINES) + '\n')

# ------------------------------------------------------------------ plots
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
a, b = legs[2]
s = (tg >= a + 5) & (tg <= a + 25)
fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
ax[0].plot(tg[s], cnv[s], label='DWB /cmd_vel_nav', lw=1); ax[0].plot(tg[s], cvv[s], label='smoother /cmd_vel', lw=1)
ax[0].plot(tg[s], v[s], label='Gazebo truth /odom', lw=1.2, c='k'); ax[0].set_ylabel('v [m/s]'); ax[0].legend(fontsize=8)
ax[1].plot(tg[s], cnw[s], lw=1); ax[1].plot(tg[s], cvw[s], lw=1); ax[1].plot(tg[s], w[s], lw=1.2, c='k'); ax[1].set_ylabel('w [rad/s]')
ax[2].plot(tg[s], al[s], label='left wheel accel', lw=1); ax[2].plot(tg[s], ar[s], label='right wheel accel', lw=1)
ax[2].axhline(WHEEL_LIMIT, c='r', ls='--', lw=0.8); ax[2].axhline(-WHEEL_LIMIT, c='r', ls='--', lw=0.8, label='plugin wheel limit')
ax[2].set_ylabel('m/s^2'); ax[2].set_xlabel('sim time [s]'); ax[2].legend(fontsize=8)
ax[0].set_title('Loading -> Inventory, 20 s excerpt: command chain vs actual motion')
fig.savefig(OUT / 'command_chain_excerpt.png', dpi=110, bbox_inches='tight')

fig, ax = plt.subplots(1, 3, figsize=(14, 3.8))
ax[0].hist(np.abs(a_lin[moving]), bins=80, range=(0, 3)); ax[0].axvline(1.0, c='r', ls='--', label='wheel limit'); ax[0].axvline(2.5, c='g', ls='--', label='DWB/smoother'); ax[0].set_title('|body linear accel| [m/s^2]'); ax[0].set_yscale('log'); ax[0].legend(fontsize=7)
ax[1].hist(np.abs(a_ang[moving]), bins=80, range=(0, 15)); ax[1].axvline(3.2, c='g', ls='--'); ax[1].axvline(12.5, c='r', ls='--'); ax[1].set_title('|body angular accel| [rad/s^2]'); ax[1].set_yscale('log')
ax[2].hist(a_wheel[moving], bins=80, range=(0, 3)); ax[2].axvline(1.0, c='r', ls='--'); ax[2].set_title('max wheel rim accel [m/s^2]'); ax[2].set_yscale('log')
fig.savefig(OUT / 'acceleration_histograms.png', dpi=110, bbox_inches='tight')

fig, ax = plt.subplots(1, 2, figsize=(11, 4))
ax[0].scatter(rows[:, 4], rows[:, 1], s=10); ax[0].set_xlabel('fraction of DWB commands needing > 1.0 m/s^2 wheel accel'); ax[0].set_ylabel('high-pass std of true w [rad/s]')
ax[1].scatter(rows[:, 3], rows[:, 2], s=10); ax[1].set_xlabel('DWB command w sign reversals / 2 s'); ax[1].set_ylabel('true w sign reversals / 2 s')
fig.savefig(OUT / 'oscillation_association.png', dpi=110, bbox_inches='tight')
print('done')
