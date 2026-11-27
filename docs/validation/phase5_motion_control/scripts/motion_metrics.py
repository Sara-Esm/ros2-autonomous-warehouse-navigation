#!/usr/bin/env python3
"""Motion-quality metrics for a recorded mission (identical definitions for every Phase 5 run).

Usage: motion_metrics.py <label>=<run_dir> [...]   (writes nothing; prints one block per run)

Time base: sim time (bag receive wall time mapped through /clock). Window: each navigation leg, from the
mission log 'Navigating to' to 'ARRIVED'. Signals: /cmd_vel_nav (DWB output), /cmd_vel (smoother output,
what the robot receives), /odom twist (Gazebo truth, odometry_source WORLD).

- Oscillation (undesirable): an angular reversal = sign change of commanded omega where |omega| >= 0.10
  rad/s on both sides and the two opposite-sign extremes are < 1.0 s sim apart. Reported per minute of
  navigation and split by phase of motion: CRUISE (|v_cmd| >= 0.10 m/s) vs SLOW/IN-PLACE (|v_cmd| < 0.10).
- Legitimate turning: rotation segments of one sign lasting >= 1.0 s sim (counted, not penalised).
- Oscillation amplitude: RMS of the high-pass commanded omega (omega minus its centred 1 s sim moving mean)
  during CRUISE.
- Abrupt angular motion: per-command |delta omega| p95 / max on /cmd_vel; truth angular acceleration p95 /
  max (odom twist differentiated, 0.1 s sim smoothing).
- Rotation economy per leg: total |rotation| (integral |omega_truth| dt) / |net heading change|; and total
  rotation in degrees.
- Command-to-motion latency: lag (sim) maximising the cross-correlation of /cmd_vel angular and truth
  angular velocity (both resampled at 100 Hz); linear tracking RMS error |v_cmd(t - lag) - v_truth(t)|.
- Travel distance per leg (truth).
- Final approach: time from first entry within 0.25 m of the leg's final position to ARRIVED (sim), and
  rotation performed in that window.
"""
import re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(run):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    clk, nav, cmd, od = [], [], [], []
    while r.has_next():
        t, d, ts = r.read_next(); w = ts * 1e-9
        if t == '/clock':
            m = deserialize_message(d, ty[t]); clk.append((w, m.clock.sec + m.clock.nanosec * 1e-9))
        elif t in ('/cmd_vel_nav', '/cmd_vel'):
            m = deserialize_message(d, ty[t]); (nav if t == '/cmd_vel_nav' else cmd).append((w, m.linear.x, m.angular.z))
        elif t == '/odom':
            m = deserialize_message(d, ty[t]); p = m.pose.pose
            od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x, p.position.y, yaw(p.orientation),
                       m.twist.twist.linear.x, m.twist.twist.angular.z))
    clk = np.array(clk)
    sim = lambda x: np.interp(x, clk[:, 0], clk[:, 1])
    nav, cmd, od = np.array(nav), np.array(cmd), np.array(od)
    nav[:, 0] = sim(nav[:, 0]); cmd[:, 0] = sim(cmd[:, 0])
    return sim, nav, cmd, od


def reversals(c, v_thr=0.10, w_thr=0.10, max_dt=1.0):
    """Count angular reversals; return (cruise_count, slow_count)."""
    t, v, w = c[:, 0], c[:, 1], c[:, 2]
    sig = np.where(w >= w_thr, 1, np.where(w <= -w_thr, -1, 0))
    idx = np.nonzero(sig)[0]
    cr = sl = 0
    for a, b in zip(idx, idx[1:]):
        if sig[a] != sig[b] and t[b] - t[a] < max_dt:
            if abs(v[b]) >= v_thr:
                cr += 1
            else:
                sl += 1
    return cr, sl


def turns(c, w_thr=0.10, min_dur=1.0):
    t, w = c[:, 0], c[:, 2]
    s = np.where(w >= w_thr, 1, np.where(w <= -w_thr, -1, 0))
    n, start = 0, None
    for i in range(len(s)):
        if start is None and s[i] != 0:
            start = i
        elif start is not None and s[i] != s[start]:
            n += t[i - 1] - t[start] >= min_dur; start = i if s[i] != 0 else None
    return n


def main():
    for arg in sys.argv[1:]:
        label, run = arg.split('=', 1)
        sim, nav, cmd, od = load(run)
        legs = []
        ev = [(float(m.group(1)), m.group(2), m.group(3)) for l in open(run + '/mission.log')
              for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)(.*)', l)] if m]
        for (w0, k0, n0), (w1, k1, _) in zip(ev, ev[1:]):
            if k0 == 'Navigating to' and k1 == 'ARRIVED':
                legs.append((n0.split(':')[0].strip(), float(sim(w0)), float(sim(w1))))
        tot = dict(nav_s=0, rev_cr=0, rev_sl=0, rev_cr_nav=0, rev_sl_nav=0, turns=0, dist=0, rot=0)
        hp_all, dw_all, aacc_all, lag_all, verr_all = [], [], [], [], []
        print(f'{label}: {len(legs)} legs')
        for name, t0, t1 in legs:
            c = cmd[(cmd[:, 0] >= t0) & (cmd[:, 0] <= t1)]; n = nav[(nav[:, 0] >= t0) & (nav[:, 0] <= t1)]
            o = od[(od[:, 0] >= t0) & (od[:, 0] <= t1)]
            rc, rs = reversals(c); nrc, nrs = reversals(n)
            dist = float(np.sum(np.hypot(np.diff(o[:, 1]), np.diff(o[:, 2]))))
            rot = float(np.sum(np.abs(o[1:, 5]) * np.diff(o[:, 0])))
            net = abs(float(np.angle(np.exp(1j * (o[-1, 3] - o[0, 3])))))
            # high-pass omega in cruise
            tg = np.arange(t0, t1, 0.01); wg = np.interp(tg, c[:, 0], c[:, 2]); vg = np.interp(tg, c[:, 0], c[:, 1])
            k = np.ones(101) / 101; hp = wg - np.convolve(wg, k, mode='same'); hp_all.append(hp[np.abs(vg) >= 0.10][50:-50])
            dw_all.append(np.abs(np.diff(c[:, 2])))
            wt = np.interp(tg, o[:, 0], o[:, 5]); ws = np.convolve(wt, np.ones(10) / 10, mode='same')
            aacc_all.append(np.abs(np.diff(ws)) / 0.01)
            # latency: cross-correlate commanded vs truth angular velocity (lags 0..300 ms)
            best = max(range(0, 31), key=lambda L: np.corrcoef(wg[:len(wg) - L], wt[L:])[0, 1] if len(wg) > L + 10 else -1)
            lag_all.append(best * 0.01)
            vt = np.interp(tg, o[:, 0], o[:, 4]); verr_all.append((vg[:len(vg) - best] - vt[best:]))
            # final approach
            fx, fy = o[-1, 1], o[-1, 2]; near = np.nonzero(np.hypot(o[:, 1] - fx, o[:, 2] - fy) <= 0.25)[0]
            fa_t = t1 - o[near[0], 0] if len(near) else float('nan')
            fa_rot = float(np.degrees(np.sum(np.abs(o[near[0] + 1:, 5]) * np.diff(o[near[0]:, 0])))) if len(near) else float('nan')
            tn = turns(c)
            print(f'  {name:30s} {t1-t0:6.1f} s sim  dist {dist:5.2f} m  rotation {np.degrees(rot):6.0f} deg (net {np.degrees(net):4.0f}, ratio {rot/max(net,0.05):5.1f})'
                  f'  reversals cruise/slow {rc}/{rs} (DWB {nrc}/{nrs})  turns {tn}  final approach {fa_t:4.1f} s, {fa_rot:5.0f} deg')
            tot['nav_s'] += t1 - t0; tot['rev_cr'] += rc; tot['rev_sl'] += rs; tot['rev_cr_nav'] += nrc; tot['rev_sl_nav'] += nrs
            tot['turns'] += tn; tot['dist'] += dist; tot['rot'] += rot
        m = tot['nav_s'] / 60
        hp = np.concatenate(hp_all); dw = np.concatenate(dw_all); aa = np.concatenate(aacc_all); ve = np.concatenate(verr_all)
        print(f'  TOTAL nav {tot["nav_s"]:.0f} s sim, distance {tot["dist"]:.2f} m, rotation {np.degrees(tot["rot"]):.0f} deg')
        print(f'  angular reversals per nav-minute: cruise {tot["rev_cr"]/m:.2f}, slow/in-place {tot["rev_sl"]/m:.2f} '
              f'(counts {tot["rev_cr"]}/{tot["rev_sl"]}; DWB output {tot["rev_cr_nav"]}/{tot["rev_sl_nav"]}); sustained turns {tot["turns"]}')
        print(f'  cruise omega high-pass RMS {np.sqrt(np.mean(hp**2)):.3f} rad/s; |d omega| per /cmd_vel step p95 {np.percentile(dw,95):.3f}, max {dw.max():.3f} rad/s; '
              f'truth angular accel p95 {np.percentile(aa,95):.2f}, max {aa.max():.2f} rad/s^2')
        print(f'  command-to-motion lag (angular, per leg) {", ".join(f"{x*1e3:.0f}" for x in lag_all)} ms sim; linear tracking RMS error {np.sqrt(np.mean(ve**2)):.4f} m/s')


if __name__ == '__main__':
    main()
