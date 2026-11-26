#!/usr/bin/env python3
"""Evaluate the Phase 1 (station frame correction) validation mission.

Usage: phase1_evaluate.py <run_dir> <baseline_run_dir> <workspace>
Ground truth: /odom (Gazebo world pose; verified in the v1.0.1 diagnostics).
Writes <run_dir>/analysis/{phase1_results.txt, *.csv, *.png}.
"""
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RUN, BASE, WS = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
sys.path.insert(0, str(WS / 'src/warehouse_bringup'))
from warehouse_bringup import stations as st  # noqa: E402

OUT = RUN / 'analysis'
OUT.mkdir(exist_ok=True)
L = []


def P(s=''):
    L.append(s)
    print(s)


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrapd(a):
    return (a + 180) % 360 - 180


def stamp(h):
    return h.sec + h.nanosec * 1e-9


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(RUN / 'bag'), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
D = defaultdict(list)
while r.has_next():
    t, d, ts = r.read_next()
    D[t].append((ts * 1e-9, deserialize_message(d, ty[t])))
P('# Phase 1 validation: station frame correction')
P('topics: ' + ', '.join(f'{k} {len(v)}' for k, v in sorted(D.items())))

clk = np.array([(w, stamp(m.clock)) for w, m in D['/clock']])
od = np.array([(w, stamp(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
                math.degrees(yaw_q(m.pose.pose.orientation)), m.twist.twist.linear.x, m.twist.twist.angular.z)
               for w, m in D['/odom']])
am = np.array([(w, stamp(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
                math.degrees(yaw_q(m.pose.pose.orientation))) for w, m in D['/amcl_pose']])
cmd = np.array([(w, m.linear.x, m.angular.z) for w, m in D['/cmd_vel']])
cmdn = np.array([(w, m.linear.x, m.angular.z) for w, m in D['/cmd_vel_nav']])
meta = (RUN / 'meta.txt').read_text()
ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1))
me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
rc = int(re.search(r'mission_exit_code: (\d+)', meta).group(1))


def events(path):
    ev = []
    for line in open(path, errors='ignore'):
        m = re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (.*)', line)
        if m:
            ev.append((float(m.group(1)), m.group(2)))
    return ev


EV = events(RUN / 'mission.log')
P(f'\nmission exit code {rc}; mission wall duration {me-ms:.0f} s')
for w, t in EV:
    if any(k in t for k in ('WAREHOUSE MISSION', 'Successful tasks', 'Failed tasks', 'Unattempted', 'HALTED',
                            'FAILED', 'RETRY', 'STOP')):
        P(f'  {w-ms:7.1f} s  {t}')


def odom_at_wall(w):
    i = min(np.searchsorted(od[:, 0], w), len(od) - 1)
    return od[i]


def odom_at_stamp(s):
    x = np.interp(s, od[:, 1], od[:, 2]); y = np.interp(s, od[:, 1], od[:, 3])
    i = min(np.searchsorted(od[:, 1], s), len(od) - 1)
    return x, y, od[i, 4]


# ---------------------------------------------------------------- station positioning
BASE_ERR = {'Home Charging Dock': (0.504, 9.6), 'Loading Zone': (0.404, 9.5),
            'Inventory Scan Station': (0.176, 8.9), 'Dispatch Zone': (0.316, -2.4)}
wp_by_name = {wp['name']: wp for wp in st.WAYPOINTS}
arrivals = [(w, t.split(': ', 1)[1]) for w, t in EV if t.startswith('ARRIVED: ')]
confirms = [w for w, t in EV if t.startswith('Correct marker')]
verified = [t.split(': ', 1)[1] for w, t in EV if t.startswith('VISION VERIFIED: ')]
P('\n## Station positioning (truth pose at the ARRIVED log line; world frame)')
P('| Step | Station | Intended world (x, y, yaw) | True stop (x, y, yaw) | Position error | Heading error | '
  'v1.0.1 position / heading | AMCL estimate (map) | Arrival->confirm | Marker |')
P('|---|---|---|---|---|---|---|---|---|---|')
rows = []
for k, (w, name) in enumerate(arrivals):
    wp = wp_by_name[name]
    ix, iy, iyaw = st.STATIONS_WORLD[wp['station']]
    o = odom_at_wall(w)
    perr = math.hypot(o[2] - ix, o[3] - iy)
    herr = wrapd(o[4] - iyaw)
    a = am[np.searchsorted(am[:, 0], w) - 1]
    conf = next((c - w for c in confirms if c >= w), float('nan'))
    base = BASE_ERR.get(name.replace('Return to ', ''), None)
    bs = f'{base[0]:.3f} m / {base[1]:+.1f}°' if base and not name.startswith('Return') else (
        'not reached (crash)' if name.startswith('Return') else '-')
    P(f'| {k+1} | {name} | ({ix:+.3f}, {iy:+.3f}, {iyaw:+.0f}°) | ({o[2]:+.3f}, {o[3]:+.3f}, {o[4]:+.1f}°) | '
      f'**{perr:.3f} m** | **{herr:+.1f}°** | {bs} | ({a[2]:+.3f}, {a[3]:+.3f}, {a[4]:+.1f}°) | {conf:.2f} s | '
      f'{"verified" if name in verified else "NOT verified"} |')
    rows.append((k + 1, perr, herr, conf))
rows = np.array(rows)
if len(rows):
    P(f'\nposition error: max {rows[:,1].max():.3f} m, mean {rows[:,1].mean():.3f} m; all <= 0.15 m: {bool(np.all(rows[:,1] <= 0.15))}')
    P(f'heading error: mean {rows[:,2].mean():+.2f}°, mean |err| {np.abs(rows[:,2]).mean():.2f}°, range {rows[:,2].min():+.1f}..{rows[:,2].max():+.1f}°')
    P(f'v1.0.1 baseline (4 stations): position mean {np.mean([v[0] for v in BASE_ERR.values()]):.3f} m, '
      f'max {max(v[0] for v in BASE_ERR.values()):.3f} m; heading mean {np.mean([v[1] for v in BASE_ERR.values()]):+.2f}°')
    first4 = rows[:4]
    P(f'same 4 stations now: position mean {first4[:,1].mean():.3f} m, max {first4[:,1].max():.3f} m; heading mean {first4[:,2].mean():+.2f}°')
np.savetxt(OUT / 'station_errors.csv', rows, delimiter=',', fmt='%.4f', header='step,pos_err_m,heading_err_deg,arrival_to_confirm_s')

# ---------------------------------------------------------------- AMCL residuals (fixed calibration A)
def amcl_world_err(amcl_xy_yaw, truth):
    wx, wy, wyaw = st.map_to_world(*amcl_xy_yaw)
    return math.hypot(wx - truth[0], wy - truth[1]), wrapd(wyaw - truth[2])


res = []
for w, s, x, y, yaw in am:
    tx, ty, tyaw = odom_at_stamp(s)
    e, h = amcl_world_err((x, y, yaw), (tx, ty, tyaw))
    res.append((s, tx, ty, e, h, math.hypot(x - tx, y - ty)))
res = np.array(res)
P('\n## AMCL residual vs truth (AMCL map pose mapped to world with the fixed calibration; no refit)')
P(f'this run (n={len(res)}): median {np.median(res[:,3])*100:.1f} cm, p95 {np.percentile(res[:,3],95)*100:.1f} cm, '
  f'max {res[:,3].max()*100:.1f} cm; heading median |{np.median(np.abs(res[:,4])):.2f}|°, p95 {np.percentile(np.abs(res[:,4]),95):.2f}°')
P(f'  (raw, map frame taken as world: median {np.median(res[:,5])*100:.1f} cm; expected to stay large, the frames still differ)')
b = np.loadtxt(BASE / 'analysis' / 'amcl_vs_truth.csv', delimiter=',')
be = []
for row in b:
    tx, ty = row[5], row[6]
    ax, ay = tx + row[2], ty + row[3]
    wx, wy, _ = st.map_to_world(ax, ay, 0.0)
    be.append(math.hypot(wx - tx, wy - ty))
be = np.array(be)
P(f'v1.0.1 baseline, same fixed calibration (n={len(be)}): median {np.median(be)*100:.1f} cm, p95 {np.percentile(be,95)*100:.1f} cm, max {be.max()*100:.1f} cm')
for lo, hi in [(-11, -6), (-6, -2), (-2, 2), (2, 6), (6, 11)]:
    s_ = (res[:, 2] >= lo) & (res[:, 2] < hi)
    if s_.sum():
        P(f'  true y in [{lo:+3d},{hi:+3d}): median {np.median(res[s_,3])*100:.1f} cm (n={s_.sum()})')
np.savetxt(OUT / 'amcl_residual.csv', res, delimiter=',', fmt='%.5f',
           header='sim_stamp,true_x,true_y,err_world_fixed_calib_m,heading_err_deg,raw_err_m')

# ---------------------------------------------------------------- recoveries and timeouts
launch = (RUN / 'launch.log').read_text(errors='ignore').splitlines()
cat = {'ack_timeout': 'acknowledge goal request', 'ctrl_miss': 'Control loop missed', 'bt_tick': 'Behavior Tree tick rate',
       'spin': 'Running spin', 'backup': 'Running backup', 'wait': 'Running wait', 'no_valid': 'No valid trajectories',
       'no_progress': 'Failed to make progress', 'plan_fail': 'failed to create plan', 'fp_abort': '[follow_path] [ActionServer] Aborting',
       'crash': 'process has died', 'assert': 'Assertion'}
cnt = defaultdict(int)
for line in launch:
    m = re.search(r'\[(\d{10}\.\d+)\]', line)
    tw = float(m.group(1)) if m else None
    for k, p in cat.items():
        if p in line and (k in ('crash', 'assert') or (tw and ms <= tw <= me)):
            cnt[k] += 1
P('\n## Recoveries, timeouts and runtime errors (mission window; crash lines over whole log)')
P('  ' + ', '.join(f'{k} {cnt[k]}' for k in cat))
acks = [m.group(1) for line in launch for m in [re.search(r'\[(\d{10}\.\d+)\].*acknowledge goal request for (\w+)', line)] if m and ms <= float(m.group(1)) <= me]
kinds = [re.search(r'request for (\w+)', line).group(1) for line in launch if 'acknowledge goal request' in line]
P(f'  ack timeouts by action (whole log): ' + str({k: kinds.count(k) for k in sorted(set(kinds))}))
died = [line for line in launch if 'process has died' in line]
for d_ in died:
    m = re.search(r'\[(\d{10}\.\d+)\]', d_)
    P('  ' + re.sub(r'\x1b\[[0-9;]*m', '', d_)[:140])


def status_events(topic):
    last, ev = {}, defaultdict(list)
    for w, m in D[topic]:
        for s_ in m.status_list:
            gid = bytes(s_.goal_info.goal_id.uuid).hex()[:8]
            if last.get(gid) != s_.status:
                last[gid] = s_.status
                ev[gid].append((w, s_.status))
    return ev


for a in ['navigate_to_pose', 'follow_path', 'compute_path_to_pose', 'spin', 'backup', 'wait', 'drive_on_heading']:
    ev = status_events(f'/{a}/_action/status')
    fin = defaultdict(int)
    for gid, e in ev.items():
        fin[e[-1][1]] += 1
    P(f'  {a:22s} goals {len(ev):4d}; final status counts {dict(fin)} (4=succeeded, 5=canceled, 6=aborted)')
for a in ['spin', 'backup', 'wait']:
    for gid, e in status_events(f'/{a}/_action/status').items():
        o = odom_at_wall(e[0][0])
        P(f'    {a} at {e[0][0]-ms:.1f} s wall, truth ({o[2]:+.2f}, {o[3]:+.2f}), lasted {e[-1][0]-e[0][0]:.1f} s, final {e[-1][1]}')

# ---------------------------------------------------------------- timing
rtf = [(np.interp(w0 + 10, clk[:, 0], clk[:, 1]) - np.interp(w0, clk[:, 0], clk[:, 1])) / 10
       for w0 in np.arange(ms, min(me, clk[-1, 0]) - 10, 10)]
cam = np.array([w for w, _ in D['/camera/camera_info']])
P('\n## Timing')
P(f'  RTF during mission (10 s windows): mean {np.mean(rtf):.3f}, min {np.min(rtf):.3f}, max {np.max(rtf):.3f}')
if len(cam) > 2:
    span = np.interp(cam[-1], clk[:, 0], clk[:, 1]) - np.interp(cam[0], clk[:, 0], clk[:, 1])
    P(f'  camera frames: {len(cam)/span:.2f} Hz sim, median wall gap {np.median(np.diff(cam)):.2f} s')
cpu = (RUN / 'cpu_top.log').read_text(errors='ignore')
idle = [float(x) for x in re.findall(r'%Cpu\(s\):.*?([\d.]+) id', cpu)]
loads = [float(x) for x in re.findall(r'load average: ([\d.]+)', cpu)]
P(f'  CPU busy median {100-np.median(idle):.0f}%, load average median {np.median(loads):.1f}')

# ---------------------------------------------------------------- planned vs executed
P('\n## Planned vs executed (per leg)')
starts = [w for w, t in EV if t.startswith('Navigating to')]
ends = [w for w, t in EV if t.startswith('ARRIVED') or 'HALTED' in t]
plans = [(w, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])) for w, m in D['/plan'] if len(m.poses) > 1]


def plen(xy):
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0


legs = []
for k, (a, b_) in enumerate(zip(starts, ends)):
    sel = (od[:, 0] >= a) & (od[:, 0] <= b_)
    ex = plen(od[sel][:, 2:4])
    fp = next((xy for w, xy in plans if w >= a), None)
    P(f'  leg {k+1}: {b_-a:6.1f} s wall, first plan {plen(fp) if fp is not None else float("nan"):.2f} m, executed {ex:.2f} m')
    legs.append((a, b_, fp))

# ---------------------------------------------------------------- completion and post-completion motion
P('\n## Completion and motion after the final report')
nav = status_events('/navigate_to_pose/_action/status')
last_nav = sorted(nav.values(), key=lambda e: e[0][0])[-1] if nav else None
comp = next((w for w, t in EV if 'WAREHOUSE MISSION' in t), None)
if last_nav:
    succ = next((w for w, s_ in last_nav if s_ == 4), None)
    P(f'  final NavigateToPose terminal status {last_nav[-1][1]} at {last_nav[-1][0]-ms:.2f} s')
    if succ and comp:
        fin_arr = arrivals[-1][0] if arrivals else None
        conf5 = next((c for c in confirms if fin_arr and c >= fin_arr), None)
        P(f'  final success {succ-ms:.2f} s -> ARRIVED log {fin_arr-ms:.2f} s -> marker confirm {(conf5 or float("nan"))-ms:.2f} s -> '
          f'MISSION COMPLETE {comp-ms:.2f} s -> process exit {me-ms:.2f} s (wall, rel. mission start)')
        for label, t0, t1 in [('final success -> COMPLETE', succ, comp), ('COMPLETE -> exit', comp, me),
                              ('exit -> +60 s (recording)', me, me + 60)]:
            so = od[(od[:, 0] >= t0) & (od[:, 0] <= t1)]
            c1 = cmd[(cmd[:, 0] >= t0) & (cmd[:, 0] <= t1)]
            c2 = cmdn[(cmdn[:, 0] >= t0) & (cmdn[:, 0] <= t1)]
            if len(so) < 2:
                P(f'  {label}: no odometry'); continue
            P(f'  {label}: {so[-1,0]-so[0,0]:.1f} s observed, truth displacement {1000*math.hypot(so[-1,2]-so[0,2], so[-1,3]-so[0,3]):.1f} mm, '
              f'yaw change {wrapd(so[-1,4]-so[0,4]):+.3f}°, max |v| {1000*np.abs(so[:,5]).max():.2f} mm/s, max |w| {np.abs(so[:,6]).max():.4f} rad/s; '
              f'nonzero /cmd_vel {int(np.sum((np.abs(c1[:,1])>1e-4)|(np.abs(c1[:,2])>1e-4))) if len(c1) else 0} of {len(c1)}, '
              f'nonzero /cmd_vel_nav {int(np.sum((np.abs(c2[:,1])>1e-4)|(np.abs(c2[:,2])>1e-4))) if len(c2) else 0} of {len(c2)}')
        mv = od[(od[:, 0] > succ - 5) & ((np.abs(od[:, 5]) > 0.002) | (np.abs(od[:, 6]) > 0.005))]
        P(f'  last true motion (>2 mm/s or >0.005 rad/s) at {mv[-1,0]-succ:+.2f} s relative to final nav success' if len(mv) else '  no motion')
        for a in ['navigate_to_pose', 'follow_path', 'spin', 'backup', 'wait']:
            act = [w for w, m in D[f'/{a}/_action/status'] if w > comp and any(s_.status in (1, 2, 3) for s_ in m.status_list)]
            P(f'  {a}: active-status messages after COMPLETE: {len(act)}')
(OUT / 'phase1_results.txt').write_text('\n'.join(L) + '\n')

# ---------------------------------------------------------------- plot
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
img = cv2.imread(str(WS / 'src/warehouse_bringup/maps/warehouse_map.pgm'), cv2.IMREAD_UNCHANGED)
H, W = img.shape
fig, ax = plt.subplots(figsize=(9, 12))
ax.imshow(img, cmap='gray', extent=[-7.56, -7.56 + W * .05, -10.8, -10.8 + H * .05], origin='upper', vmin=0, vmax=255)
ax.plot(am[:, 2], am[:, 3], 'r--', lw=0.8, label='AMCL estimate (map frame)')
for a, b_, fp in legs:
    if fp is not None:
        ax.plot(fp[:, 0], fp[:, 1], c='orange', lw=0.9)
ax.plot([], [], c='orange', label='first global plan per leg (map frame)')
for wp in st.WAYPOINTS:
    ax.plot(wp['x'], wp['y'], 'k*', ms=12)
    ox_, oy_ = st.STATIONS_WORLD[wp['station']][:2]
    ax.plot(ox_, oy_, 'kx', ms=8)
ax.plot([], [], 'k*', label='corrected goal (map frame)'); ax.plot([], [], 'kx', label='v1.0.1 goal (world coords used as map)')
ax.legend(loc='lower left', fontsize=8); ax.set_title('Phase 1: map-frame view (plans, AMCL, goals)')
fig.savefig(OUT / 'phase1_map_frame.png', dpi=110, bbox_inches='tight')
fig, ax = plt.subplots(figsize=(9, 12))
ax.plot(od[:, 2], od[:, 3], lw=1, label='true trajectory (world)')
for wp in st.WAYPOINTS:
    ix, iy, iyaw = st.STATIONS_WORLD[wp['station']]
    ax.plot(ix, iy, 'g*', ms=12)
for k, (w, name) in enumerate(arrivals):
    o = odom_at_wall(w)
    ax.plot(o[2], o[3], 'bo', ms=5)
    ax.annotate(f'{rows[k,1]*100:.0f} cm', (o[2], o[3]), textcoords='offset points', xytext=(6, -12), fontsize=8)
ax.plot([], [], 'g*', label='intended approach pose (world)'); ax.plot([], [], 'bo', label='true stop at ARRIVED')
ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(fontsize=8); ax.set_title('Phase 1: world-frame stops vs intended poses')
fig.savefig(OUT / 'phase1_world_stops.png', dpi=110, bbox_inches='tight')
print('done')
