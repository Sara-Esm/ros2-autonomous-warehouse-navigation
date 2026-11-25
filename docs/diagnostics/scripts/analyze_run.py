#!/usr/bin/env python3
"""Analyze the instrumented diagnostic mission (read-only on inputs)."""
import bisect
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RUN = Path(sys.argv[1])
WS = Path(sys.argv[2])
STATIC = RUN.parent / 'static'
OUT = RUN / 'analysis'
OUT.mkdir(exist_ok=True)
REPORT = []


def P(*a):
    s = ' '.join(str(x) for x in a)
    REPORT.append(s)
    print(s)


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def st(msg):
    return msg.sec + msg.nanosec * 1e-9


# ------------------------------------------------------------------ load bag
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(RUN / 'bag'), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msgcls = {k: get_message(v) for k, v in types.items()}
D = defaultdict(list)   # topic -> [(recv_wall_s, msg)]
while reader.has_next():
    topic, data, t = reader.read_next()
    D[topic].append((t * 1e-9, deserialize_message(data, msgcls[topic])))
P('# Topics recorded:', {k: len(v) for k, v in sorted(D.items())})

# wall<->sim mapping from /clock
clk = np.array([(w, st(m.clock)) for w, m in D['/clock']])
def sim_at_wall(w):
    return float(np.interp(w, clk[:, 0], clk[:, 1]))
def wall_at_sim(s):
    return float(np.interp(s, clk[:, 1], clk[:, 0]))

# ------------------------------------------------------------------ mission log events
mission = (RUN / 'mission.log').read_text(errors='ignore').splitlines()
EV = []
for l in mission:
    m = re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (.*)', l)
    if m:
        EV.append((float(m.group(1)), m.group(2)))
segments = []  # (name, nav_start_wall, arrived_wall, confirm_wall)
cur = None
for w, txt in EV:
    if txt.startswith('Navigating to'):
        cur = {'name': txt.split(':')[0].replace('Navigating to ', ''), 'start': w, 'txt': txt}
        segments.append(cur)
    elif txt.startswith('ARRIVED') and cur:
        cur['arrived'] = w
    elif txt.startswith('Correct marker') and cur:
        cur['confirm'] = w
    elif txt.startswith('TASK COMPLETE') and cur:
        cur['task_complete'] = w
    elif txt.startswith('Navigation FAILED') and cur:
        cur['failed'] = w
complete_wall = next((w for w, t in EV if 'WAREHOUSE MISSION' in t), None)
halt_wall = next((w for w, t in EV if 'MISSION HALTED' in t), None)
GOALS = {'Home Charging Dock': (-3.678, 9.3, 90), 'Loading Zone': (1.803, 9.3, 90),
         'Inventory Scan Station': (1.801, -9.0, -90), 'Dispatch Zone': (-3.630, -9.3, -90),
         'Return to Home Charging Dock': (-3.678, 9.3, 90)}
MARK = {'Home Charging Dock': (-3.678265, 9.97, 0), 'Loading Zone': (1.802768, 9.97, 1),
        'Inventory Scan Station': (1.800570, -9.97, 2), 'Dispatch Zone': (-3.629830, -9.97, 3),
        'Return to Home Charging Dock': (-3.678265, 9.97, 0)}

# ------------------------------------------------------------------ series
odom = [(w, st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
         yaw_q(m.pose.pose.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z)
        for w, m in D['/odom']]
odom = np.array(odom)
amcl = np.array([(w, st(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
                  yaw_q(m.pose.pose.orientation), m.pose.covariance[0], m.pose.covariance[7],
                  m.pose.covariance[35]) for w, m in D['/amcl_pose']])
cmdn = np.array([(w, m.linear.x, m.angular.z) for w, m in D['/cmd_vel_nav']]) if D['/cmd_vel_nav'] else np.zeros((0, 3))
cmd = np.array([(w, m.linear.x, m.angular.z) for w, m in D['/cmd_vel']]) if D['/cmd_vel'] else np.zeros((0, 3))
tf_mo, tf_ob = [], []
for w, m in D['/tf']:
    for t in m.transforms:
        if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
            tf_mo.append((w, st(t.header.stamp), t.transform.translation.x, t.transform.translation.y,
                          yaw_q(t.transform.rotation)))
        elif t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
            tf_ob.append((w, st(t.header.stamp), t.transform.translation.x, t.transform.translation.y,
                          yaw_q(t.transform.rotation)))
tf_mo, tf_ob = np.array(tf_mo), np.array(tf_ob)
static_tf = {}
for w, m in D['/tf_static']:
    for t in m.transforms:
        static_tf[(t.header.frame_id, t.child_frame_id)] = t.transform


def odom_at_stamp(s):
    i = np.searchsorted(odom[:, 1], s)
    i = min(max(i, 1), len(odom) - 1)
    a, b = odom[i - 1], odom[i]
    f = 0 if b[1] == a[1] else (s - a[1]) / (b[1] - a[1])
    f = min(max(f, 0), 1)
    return a[2] + f * (b[2] - a[2]), a[3] + f * (b[3] - a[3]), a[4] + f * wrap(b[4] - a[4])


def interp_mo(s):
    i = np.searchsorted(tf_mo[:, 1], s, 'right') - 1
    i = min(max(i, 0), len(tf_mo) - 1)
    return tf_mo[i, 2:5]


def compose(a, b):
    x, y, t = a
    return (x + math.cos(t) * b[0] - math.sin(t) * b[1], y + math.sin(t) * b[0] + math.cos(t) * b[1], wrap(t + b[2]))


# ================================================================== 5. timing & performance
P('\n## 5. Synchronization and performance')
dw = clk[-1, 0] - clk[0, 0]
P(f'/clock: {len(clk)} msgs over {dw:.1f} s wall / {clk[-1,1]-clk[0,1]:.1f} s sim -> mean RTF {(clk[-1,1]-clk[0,1])/dw:.3f}')
win = []
for w0 in np.arange(clk[0, 0], clk[-1, 0] - 10, 10):
    s0, s1 = sim_at_wall(w0), sim_at_wall(w0 + 10)
    win.append((s1 - s0) / 10)
win = np.array(win)
P(f'RTF per 10 s wall window: min {win.min():.3f} p10 {np.percentile(win,10):.3f} median {np.median(win):.3f} max {win.max():.3f}')
np.savetxt(OUT / 'rtf_10s.csv', win, fmt='%.4f')


def rate(topic, arr_w=None, arr_s=None):
    if arr_w is None:
        arr_w = np.array([w for w, _ in D[topic]])
    if len(arr_w) < 2:
        return 'n/a'
    sim_span = sim_at_wall(arr_w[-1]) - sim_at_wall(arr_w[0])
    wall_span = arr_w[-1] - arr_w[0]
    gaps = np.diff(arr_w)
    return (f'{len(arr_w)/sim_span:6.2f} Hz sim, {len(arr_w)/wall_span:6.2f} Hz wall; wall gap p50 {np.median(gaps)*1e3:.0f} ms '
            f'p99 {np.percentile(gaps,99)*1e3:.0f} ms max {gaps.max()*1e3:.0f} ms')


for t in ['/scan', '/odom', '/camera/camera_info', '/amcl_pose', '/plan', '/local_plan', '/cmd_vel_nav',
          '/cmd_vel', '/warehouse/marker_detected', '/global_costmap/costmap', '/local_costmap/costmap']:
    if D[t]:
        P(f'  rate {t:28s} {rate(t)}')
P(f'  rate {"tf map->odom":28s} {rate(None, tf_mo[:,0])}')
P(f'  rate {"tf odom->base_footprint":28s} {rate(None, tf_ob[:,0])}')

# Age of data at reception (stamp vs current sim clock at reception)
def age(stamp_s, recv_w):
    return np.array([sim_at_wall(w) - s for s, w in zip(stamp_s, recv_w)])
scan_st = np.array([st(m.header.stamp) for _, m in D['/scan']])
scan_w = np.array([w for w, _ in D['/scan']])
for name, sts, ws in [('scan', scan_st, scan_w), ('odom', odom[:, 1], odom[:, 0]),
                      ('tf odom->base', tf_ob[:, 1], tf_ob[:, 0]), ('tf map->odom', tf_mo[:, 1], tf_mo[:, 0]),
                      ('amcl_pose', amcl[:, 1], amcl[:, 0])]:
    a = age(sts, ws)
    P(f'  stamp age at reception {name:14s}: median {np.median(a)*1e3:+7.0f} ms p95 {np.percentile(a,95)*1e3:+7.0f} ms  min {a.min()*1e3:+7.0f} ms (negative = future-dated)')
cam_st = np.array([st(m.header.stamp) for _, m in D['/camera/camera_info']])
cam_w = np.array([w for w, _ in D['/camera/camera_info']])
if len(cam_st):
    a = age(cam_st, cam_w)
    P(f'  stamp age at reception camera_info  : median {np.median(a)*1e3:+7.0f} ms p95 {np.percentile(a,95)*1e3:+7.0f} ms')
# AMCL processing latency: amcl_pose stamp equals scan stamp; delay from scan reception to amcl output (wall)
lat = []
for w, s in zip(amcl[:, 0], amcl[:, 1]):
    i = np.argmin(np.abs(scan_st - s))
    if abs(scan_st[i] - s) < 1e-6:
        lat.append(w - scan_w[i])
if lat:
    lat = np.array(lat)
    P(f'  AMCL latency scan-received -> amcl_pose published (wall): median {np.median(lat)*1e3:.0f} ms p95 {np.percentile(lat,95)*1e3:.0f} ms max {lat.max()*1e3:.0f} ms (n={len(lat)})')
# CPU from top
cpu_txt = (RUN / 'cpu_top.log').read_text(errors='ignore')
blocks = cpu_txt.split('=== ')[1:]
proc_cpu = defaultdict(list)
idle = []
loads = []
for b in blocks:
    for l in b.splitlines():
        m = re.search(r'%Cpu\(s\):.*?([\d.]+) id', l)
        if m:
            idle.append(float(m.group(1)))
        m = re.search(r'load average: ([\d.]+)', l)
        if m:
            loads.append(float(m.group(1)))
        f = l.split()
        if len(f) >= 12 and f[0].isdigit():
            try:
                proc_cpu[f[11]].append(float(f[8]))
            except ValueError:
                pass
P(f'CPU (top, {len(blocks)} samples, 8 logical CPUs = 800%): total busy median {100-np.median(idle):.0f}% p90 {100-np.percentile(idle,10):.0f}% ; load avg 1-min median {np.median(loads):.1f} max {max(loads):.1f}')
tops = sorted(((np.mean(v + [0] * (len(blocks) - len(v))), k) for k, v in proc_cpu.items()), reverse=True)[:10]
for avg, k in tops:
    v = proc_cpu[k]
    P(f'    {k:28s} mean {avg:6.1f}%  p90 {np.percentile(v,90):6.1f}% (of one core)')
mem = re.findall(r'MiB Mem :\s+([\d.]+) total,\s+([\d.]+) free,\s+([\d.]+) used', cpu_txt)
swp = re.findall(r'MiB Swap:\s+([\d.]+) total,\s+([\d.]+) free,\s+([\d.]+) used', cpu_txt)
if mem:
    P(f'  memory used max {max(float(m[2]) for m in mem):.0f} MiB of {mem[0][0]} ; swap used max {max(float(s[2]) for s in swp):.0f} MiB')

# Launch log warning timeline
launch = (RUN / 'launch.log').read_text(errors='ignore').splitlines()
cats = {'bt_tick': 'Behavior Tree tick rate', 'ctrl_miss': 'Control loop missed its desired rate',
        'plan_miss': 'Planner loop missed', 'ack_timeout': 'Timed out while waiting for action server to acknowledge',
        'svc_timeout': 'Node timed out while executing service call', 'no_valid': 'No valid trajectories',
        'no_progress': 'Failed to make progress', 'fp_abort': '[follow_path] [ActionServer] Aborting',
        'spin': 'Running spin', 'backup': 'Running backup', 'wait': 'Running wait',
        'tf_err': 'Extrapolation', 'tf_lookup': 'Lookup would require', 'msg_filter': 'Message Filter dropping',
        'collision': 'collision', 'qos': 'incompatible QoS'}
cat_times = defaultdict(list)
for l in launch:
    m = re.search(r'\[(\d{10}\.\d+)\]', l)
    for k, pat in cats.items():
        if pat in l and m:
            cat_times[k].append(float(m.group(1)))
ms = float(re.search(r'mission_start_wall: ([\d.]+)', (RUN / 'meta.txt').read_text()).group(1))
me = float(re.search(r'mission_end_wall: ([\d.]+)', (RUN / 'meta.txt').read_text()).group(1))
P('Launch-log warning counts (during mission window / whole run):')
for k in cats:
    v = cat_times[k]
    P(f'    {k:12s} {sum(ms <= t <= me for t in v):6d} / {len(v):6d}')
# controller miss vs RTF correlation
cm = np.array(cat_times['ctrl_miss'])
if len(cm):
    rows = []
    for w0 in np.arange(ms, me - 10, 10):
        r = (sim_at_wall(w0 + 10) - sim_at_wall(w0)) / 10
        rows.append((r, np.sum((cm >= w0) & (cm < w0 + 10)), np.sum((np.array(cat_times['bt_tick']) >= w0) & (np.array(cat_times['bt_tick']) < w0 + 10))))
    rows = np.array(rows)
    if len(rows) > 3 and rows[:, 1].std() > 0:
        P(f'  corr(RTF, controller misses per 10 s) = {np.corrcoef(rows[:,0], rows[:,1])[0,1]:+.2f}; corr(RTF, BT tick warnings) = {np.corrcoef(rows[:,0], rows[:,2])[0,1]:+.2f}')
    np.savetxt(OUT / 'rtf_vs_warnings_10s.csv', rows, fmt='%.4f', header='rtf ctrl_miss bt_tick')

# ================================================================== 1. localization
P('\n## 1. Map and localization')
truth_lines = (RUN / 'gz_truth_pose.log').read_text(errors='ignore').splitlines()
truth = []
for l in truth_lines:
    f = l.split()
    try:
        if len(f) == 7:
            truth.append([float(x) for x in f[:7]])
    except (ValueError, IndexError):
        pass
truth = np.array(truth)
if len(truth):
    # compare with odom at same wall time (only when slow, to limit timing error)
    d = []
    for w, x, y, z, r, p, yw in truth:
        i = np.searchsorted(odom[:, 0], w)
        if 0 < i < len(odom) and abs(odom[i, 5]) < 0.01 and abs(odom[i, 6]) < 0.02:
            d.append((odom[i, 2] - x, odom[i, 3] - y, math.degrees(wrap(odom[i, 4] - yw))))
    d = np.array(d)
    P(f'gz truth vs /odom (stationary samples n={len(d)}): |dxy| median {np.median(np.hypot(d[:,0],d[:,1]))*100:.2f} cm max {np.hypot(d[:,0],d[:,1]).max()*100:.2f} cm; dyaw median {np.median(np.abs(d[:,2])):.3f} deg'
      ' -> /odom is Gazebo world ground truth' if len(d) else 'no truth samples')
    P(f'  truth z range {truth[:,3].min():.3f}..{truth[:,3].max():.3f} m, |roll| max {np.degrees(np.abs(truth[:,4]).max()):.2f} deg, |pitch| max {np.degrees(np.abs(truth[:,5]).max()):.2f} deg')

# AMCL vs truth at the AMCL stamp
err = []
for w, s, x, y, t, cxx, cyy, ctt in amcl:
    gx, gy, gt = odom_at_stamp(s)
    err.append((s, w, x - gx, y - gy, wrap(t - gt), gx, gy, cxx, cyy, ctt))
err = np.array(err)
dist = np.hypot(err[:, 2], err[:, 3])
P(f'AMCL (map frame) minus Gazebo truth (world frame) at identical sim stamps, n={len(err)}:')
P(f'  position error: median {np.median(dist):.3f} m, p95 {np.percentile(dist,95):.3f} m, max {dist.max():.3f} m')
P(f'  heading error: median |{np.degrees(np.median(np.abs(err[:,4]))):.2f}| deg, p95 {np.degrees(np.percentile(np.abs(err[:,4]),95)):.2f} deg, max {np.degrees(np.abs(err[:,4]).max()):.2f} deg')
P(f'  reported 1-sigma (sqrt cov): x median {np.sqrt(np.median(err[:,7])):.3f} m, yaw median {np.degrees(np.sqrt(np.median(err[:,9]))):.2f} deg')
for lo, hi in [(-11, -6), (-6, -2), (-2, 2), (2, 6), (6, 11)]:
    sel = (err[:, 6] >= lo) & (err[:, 6] < hi)
    if sel.sum():
        P(f'  true y in [{lo:+3d},{hi:+3d}): n={sel.sum():4d} mean dx={err[sel,2].mean():+.3f} dy={err[sel,3].mean():+.3f} |e| median {np.median(dist[sel]):.3f} m, dyaw mean {np.degrees(err[sel,4].mean()):+.2f} deg')
np.savetxt(OUT / 'amcl_vs_truth.csv', err, delimiter=',', fmt='%.5f',
           header='sim_stamp,wall,ex,ey,eyaw,true_x,true_y,cov_xx,cov_yy,cov_yawyaw')
P(f'map->odom transform over run: x {tf_mo[:,2].min():+.3f}..{tf_mo[:,2].max():+.3f} m, y {tf_mo[:,3].min():+.3f}..{tf_mo[:,3].max():+.3f} m, yaw {np.degrees(tf_mo[:,4].min()):+.2f}..{np.degrees(tf_mo[:,4].max()):+.2f} deg (identity if map == world)')
jumps = np.hypot(np.diff(tf_mo[:, 2]), np.diff(tf_mo[:, 3]))
yj = np.degrees(np.abs(np.array([wrap(a) for a in np.diff(tf_mo[:, 4])])))
P(f'  map->odom step changes between consecutive AMCL updates: translation p95 {np.percentile(jumps,95)*100:.1f} cm max {jumps.max()*100:.1f} cm; yaw p95 {np.percentile(yj,95):.2f} max {yj.max():.2f} deg')

# LiDAR vs map fit with AMCL pose vs truth pose
mapimg = cv2.imread(str(WS / 'src/warehouse_bringup/maps/warehouse_map.pgm'), cv2.IMREAD_UNCHANGED)
H, W = mapimg.shape
res, ox, oy = 0.05, -7.56, -10.8
occ = ((1 - mapimg / 255.0) > 0.65).astype(np.uint8)
dmap = cv2.distanceTransform(255 - occ * 255, cv2.DIST_L2, 5) * res
gt_body = np.load(STATIC / 'gt_lidar.npy') > 0
dgt = cv2.distanceTransform(255 - gt_body.astype(np.uint8) * 255, cv2.DIST_L2, 5) * res
bl = static_tf.get(('base_footprint', 'base_link'))
ls = static_tf.get(('base_link', 'base_scan'))
off = (0.0, 0.0, 0.0)
if bl and ls:
    off = (bl.translation.x + ls.translation.x, bl.translation.y + ls.translation.y, yaw_q(ls.rotation))


def lookup(img_d, xs, ys):
    c = np.round((xs - ox) / res).astype(int)
    r = np.round(H - 1 - (ys - oy) / res).astype(int)
    ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
    out = np.full(len(xs), 1.0)
    out[ok] = img_d[r[ok], c[ok]]
    return out


fit = []
for k, (w, m) in enumerate(D['/scan']):
    if k % 3:
        continue
    s = st(m.header.stamp)
    if s < amcl[0, 1]:
        continue
    rngs = np.array(m.ranges)
    ang = m.angle_min + np.arange(len(rngs)) * m.angle_increment
    ok = np.isfinite(rngs) & (rngs > m.range_min) & (rngs < m.range_max * 0.98)
    if ok.sum() < 30:
        continue
    gx, gy, gt = odom_at_stamp(s)
    mo = interp_mo(s)
    est = compose(tuple(mo), (gx, gy, gt))  # map->base via TF (odom==truth)
    row = [s]
    for pose, dimg in ((est, dmap), ((gx, gy, gt), dmap), ((gx, gy, gt), dgt)):
        lp = compose(pose, off)
        px = lp[0] + rngs[ok] * np.cos(lp[2] + ang[ok])
        py = lp[1] + rngs[ok] * np.sin(lp[2] + ang[ok])
        dd = np.minimum(lookup(dimg, px, py), 1.0)
        row += [np.median(dd), np.mean(dd < 0.1)]
    row += [gy]
    fit.append(row)
fit = np.array(fit)
P(f'LiDAR endpoint fit (n={len(fit)} scans): median distance of scan endpoints to nearest obstacle:')
P(f'  scan @ AMCL/TF pose vs MAP     : median {np.median(fit[:,1])*100:.1f} cm, inliers(<10cm) {100*np.mean(fit[:,2]):.0f}%')
P(f'  scan @ TRUE pose     vs MAP     : median {np.median(fit[:,3])*100:.1f} cm, inliers(<10cm) {100*np.mean(fit[:,4]):.0f}%')
P(f'  scan @ TRUE pose     vs WORLD GT: median {np.median(fit[:,5])*100:.1f} cm, inliers(<10cm) {100*np.mean(fit[:,6]):.0f}%  (validates truth + GT raster)')
for lo, hi in [(-11, -6), (-6, -2), (-2, 2), (2, 6), (6, 11)]:
    sel = (fit[:, 7] >= lo) & (fit[:, 7] < hi)
    if sel.sum():
        P(f'  true y in [{lo:+3d},{hi:+3d}): AMCL-pose-vs-map inliers {100*fit[sel,2].mean():.0f}%  true-pose-vs-map inliers {100*fit[sel,4].mean():.0f}%')
np.savetxt(OUT / 'scan_fit.csv', fit, delimiter=',', fmt='%.4f')

# ================================================================== action status timeline
def status_events(topic):
    """Return {goal_id: [(wall, status)]} transitions."""
    last = {}
    ev = defaultdict(list)
    for w, m in D[topic]:
        for s in m.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()[:8]
            if last.get(gid) != s.status:
                last[gid] = s.status
                ev[gid].append((w, s.status))
    return ev


nav_ev = status_events('/navigate_to_pose/_action/status')
P('\n## Action timeline (wall s relative to mission start; status 2=executing 4=succeeded 5=canceled 6=aborted)')
nav_goals = []
for gid, ev in sorted(nav_ev.items(), key=lambda kv: kv[1][0][0]):
    P(f'  navigate_to_pose {gid}: ' + ', '.join(f'{w-ms:.1f}s->{s}' for w, s in ev))
    end = [w for w, s in ev if s in (4, 5, 6)]
    nav_goals.append((ev[0][0], end[0] if end else None, ev[-1][1]))
for a in ['follow_path', 'compute_path_to_pose', 'spin', 'backup', 'wait', 'drive_on_heading']:
    ev = status_events(f'/{a}/_action/status')
    fin = defaultdict(int)
    for gid, e in ev.items():
        fin[e[-1][1]] += 1
    P(f'  {a:22s} goals={len(ev):4d} final statuses={dict(fin)}')
spins = status_events('/spin/_action/status')
backs = status_events('/backup/_action/status')
waits = status_events('/wait/_action/status')
for name, evs in (('spin', spins), ('backup', backs), ('wait', waits)):
    for gid, e in evs.items():
        P(f'    recovery {name} at {e[0][0]-ms:.1f}s wall, final {e[-1][1]}, duration {e[-1][0]-e[0][0]:.1f}s')

# BT log: which recovery / failure nodes ran
btc = defaultdict(lambda: defaultdict(int))
bt_fail = []
for w, m in D['/behavior_tree_log']:
    for e in m.event_log:
        btc[e.node_name][e.current_status] += 1
        if e.current_status == 'FAILURE':
            bt_fail.append((w - ms, e.node_name))
P('  BT node status transitions (selected):')
for n in sorted(btc):
    if any(k in n for k in ('Spin', 'Wait', 'BackUp', 'Clear', 'FollowPath', 'ComputePath', 'Recovery', 'GoalUpdated', 'Navigate')):
        P(f'    {n:34s} {dict(btc[n])}')
P(f'  BT FAILURE events: {len(bt_fail)}; first 15: ' + '; '.join(f'{t:.0f}s {n}' for t, n in bt_fail[:15]))

# ================================================================== 2/3. planning & control per segment
P('\n## 2/3. Planning and control per navigation goal')
plans = [(w, m) for w, m in D['/plan']]
plan_w = np.array([w for w, _ in plans])


def plen(xy):
    return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))) if len(xy) > 1 else 0.0


def poly_dist(px, py, xy):
    """Distance from point to polyline."""
    a = xy[:-1]
    b = xy[1:]
    ab = b - a
    t = np.clip(((px - a[:, 0]) * ab[:, 0] + (py - a[:, 1]) * ab[:, 1]) / np.maximum((ab ** 2).sum(1), 1e-12), 0, 1)
    q = a + ab * t[:, None]
    d = np.hypot(q[:, 0] - px, q[:, 1] - py)
    i = int(np.argmin(d))
    return d[i], i


def frechet_like(xy1, xy2):
    """Max over xy2 points of distance to polyline xy1 (one-sided Hausdorff)."""
    if len(xy1) < 2 or len(xy2) < 1:
        return 0.0
    sub = xy2[::max(1, len(xy2) // 60)]
    return max(poly_dist(p[0], p[1], xy1)[0] for p in sub)


seg_rows = []
plan_csv = []
nav_goals = [(sg['start'], sg.get('arrived', halt_wall or me), 'arrived' if 'arrived' in sg else 'halted') for sg in segments]
for k, (g0, g1, final) in enumerate(nav_goals):
    if g1 is None:
        g1 = me
    seg = segments[k]
    name = seg['name'] if seg else f'goal{k}'
    gp = [(w, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])) for w, m in plans if g0 <= w <= g1 and len(m.poses) > 1]
    if not gp:
        continue
    lens = np.array([plen(xy) for _, xy in gp])
    # route switches: compare each plan with previous, only the part ahead (current plan vs previous plan)
    sw = []
    for (wa, a), (wb, b) in zip(gp[:-1], gp[1:]):
        d = frechet_like(a, b)
        sw.append(d)
        plan_csv.append((wb - ms, k, d, plen(b)))
    sw = np.array(sw) if sw else np.zeros(1)
    # executed truth path
    sel = (odom[:, 0] >= g0) & (odom[:, 0] <= g1)
    txy = odom[sel][:, 2:4]
    exec_len = plen(txy)
    start = txy[0]
    gx, gy, gyaw = GOALS.get(name, (0, 0, 0))
    straight = math.hypot(gx - start[0], gy - start[1])
    first_plan = lens[0]
    # cmd_vel analysis
    cs = cmdn[(cmdn[:, 0] >= g0) & (cmdn[:, 0] <= g1)]
    rot_inplace = np.mean((np.abs(cs[:, 1]) < 0.02) & (np.abs(cs[:, 2]) > 0.2)) if len(cs) else 0
    wz = cs[:, 2]
    sig = np.sign(wz[np.abs(wz) > 0.1])
    rev = int(np.sum(sig[1:] != sig[:-1])) if len(sig) > 1 else 0
    dur_sim = sim_at_wall(g1) - sim_at_wall(g0)
    # stop events mid-route: cmd v below 0.02 for >1 s while > 0.5 m from goal
    od = odom[sel]
    vabs = np.abs(od[:, 5])
    true_w = od[:, 6]
    sig2 = np.sign(true_w[np.abs(true_w) > 0.15])
    rev_true = int(np.sum(sig2[1:] != sig2[:-1])) if len(sig2) > 1 else 0
    # cross-track: AMCL pose (map) vs latest plan at that time
    ct = []
    for w, s, x, y, *_ in amcl[(amcl[:, 0] >= g0) & (amcl[:, 0] <= g1)]:
        j = bisect.bisect_right([p[0] for p in gp], w) - 1
        if j >= 0:
            ct.append(poly_dist(x, y, gp[j][1])[0])
    ct = np.array(ct) if ct else np.zeros(1)
    rot_total = np.sum(np.abs(true_w[:-1]) * np.diff(od[:, 1])) if len(od) > 1 else 0
    heading_change_needed = abs(math.degrees(wrap(math.radians(gyaw) - od[0, 4]))) if len(od) else 0
    P(f'\n### Goal {k}: {name} (final status {final})')
    P(f'  duration {g1-g0:.1f} s wall / {dur_sim:.1f} s sim; straight-line {straight:.2f} m; first plan {first_plan:.2f} m; executed (truth) {exec_len:.2f} m '
      f'-> executed/straight {exec_len/max(straight,1e-3):.2f}, executed/first-plan {exec_len/max(first_plan,1e-3):.2f}')
    P(f'  plans received {len(gp)} (~{len(gp)/max(dur_sim,1e-3):.2f} per sim s); plan length min {lens.min():.2f} max {lens.max():.2f}; '
      f'successive-plan deviation p50 {np.median(sw):.2f} m p95 {np.percentile(sw,95):.2f} m max {sw.max():.2f} m; switches >0.5 m: {int(np.sum(sw>0.5))}, >1.0 m: {int(np.sum(sw>1.0))}')
    P(f'  mean speed (truth) {exec_len/max(dur_sim,1e-3):.3f} m/s sim; time rotating in place (cmd) {100*rot_inplace:.0f}%; '
      f'cmd angular sign reversals {rev} ({rev/max(dur_sim,1e-3)*60:.1f}/min); true yaw-rate reversals {rev_true}')
    P(f'  total rotation executed {math.degrees(rot_total):.0f} deg (net heading change needed at goal ~{heading_change_needed:.0f} deg)')
    P(f'  cross-track error (AMCL pose to current plan): median {np.median(ct)*100:.1f} cm p95 {np.percentile(ct,95)*100:.1f} cm max {ct.max()*100:.1f} cm')
    seg_rows.append(dict(k=k, name=name, g0=g0, g1=g1, final=final, gp=gp, txy=txy))
np.savetxt(OUT / 'plan_switches.csv', np.array(plan_csv), delimiter=',', fmt='%.3f',
           header='wall_rel_s,goal_idx,deviation_from_previous_plan_m,plan_len_m')

# plans outside true walls
outside = 0
total = 0
for w, m in plans:
    for p in m.poses:
        total += 1
        x, y = p.pose.position.x, p.pose.position.y
        if x < -6.85 or x > 6.85 or y < -10.3 or y > 10.3:
            outside += 1
P(f'\nPlan poses beyond true inner wall faces: {outside} of {total}')

# cmd_vel vs cmd_vel_nav (velocity smoother) and actual
if len(cmd) and len(cmdn):
    P(f'cmd_vel_nav {len(cmdn)} msgs, cmd_vel (smoother out) {len(cmd)} msgs; max |v| nav {np.abs(cmdn[:,1]).max():.3f} out {np.abs(cmd[:,1]).max():.3f}; max |w| nav {np.abs(cmdn[:,2]).max():.3f} out {np.abs(cmd[:,2]).max():.3f}')
    dv = np.diff(cmdn[:, 1])
    P(f'  controller linear-velocity step between consecutive commands: p95 {np.percentile(np.abs(dv),95):.3f} m/s max {np.abs(dv).max():.3f} m/s')

# ================================================================== 4. ArUco
P('\n## 4. ArUco verification')
mid = [(w, m.data) for w, m in D['/warehouse/marker_id']]
mdet = [(w, m.data) for w, m in D['/warehouse/marker_detected']]
P(f'marker_detected msgs {len(mdet)} ({rate("/warehouse/marker_detected")}); camera_info {rate("/camera/camera_info")}')
for s in segments:
    if 'arrived' not in s:
        continue
    name = s['name']
    mx, my, mid_exp = MARK[name]
    a = s['arrived']
    # truth pose at arrival
    i = np.searchsorted(odom[:, 0], a)
    tx, ty, tt = odom[min(i, len(odom) - 1), 2:5]
    rng = math.hypot(mx - tx, my - ty)
    bearing = math.degrees(wrap(math.atan2(my - ty, mx - tx) - tt))
    gx, gy, gyaw = GOALS[name]
    am = amcl[np.searchsorted(amcl[:, 0], a) - 1]
    first_seen = next((w for w, d in mid if d == mid_exp and w >= s['start']), None)
    after = next((w for w, d in mid if d == mid_exp and w >= a), None)
    others = sorted(set(d for w, d in mid if s['start'] <= w <= s.get('confirm', a + 15) and d != mid_exp))
    win = [d for w, d in mdet if a <= w <= a + 3]
    P(f'  {name}: arrival truth pose ({tx:+.3f},{ty:+.3f},{math.degrees(tt):+.1f} deg) vs goal ({gx},{gy},{gyaw}) -> '
      f'true goal miss {math.hypot(tx-gx, ty-gy):.3f} m, yaw miss {math.degrees(wrap(tt-math.radians(gyaw))):+.1f} deg; AMCL said ({am[2]:+.3f},{am[3]:+.3f})')
    P(f'      true marker range {rng:.3f} m, bearing from camera axis {bearing:+.1f} deg; first sighting of ID {mid_exp} '
      f'{(first_seen - a) if first_seen else float("nan"):+.1f} s rel. arrival; first ID msg after arrival {(after - a) if after else float("nan"):.2f} s; '
      f'confirm logged {s.get("confirm", float("nan")) - a:.2f} s; detection rate in 3 s after arrival {100*np.mean(win) if win else float("nan"):.0f}%; other IDs seen on approach {others}')

# ================================================================== 6. completion
P('\n## 6. Mission completion')
if complete_wall:
    last_goal_end = nav_goals[-1][1]
    P(f'final NavigateToPose terminal status at {last_goal_end - ms:.2f} s; final marker confirm {segments[-1].get("confirm", float("nan")) - ms:.2f} s; '
      f'mission COMPLETE logged {complete_wall - ms:.2f} s; mission process end {me - ms:.2f} s (wall, rel. mission start)')
    for label, t0, t1 in [('final nav success -> mission complete', last_goal_end, complete_wall),
                          ('mission complete -> +60 s', complete_wall, complete_wall + 60)]:
        so = odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1)]
        cn = cmdn[(cmdn[:, 0] >= t0) & (cmdn[:, 0] <= t1)]
        co = cmd[(cmd[:, 0] >= t0) & (cmd[:, 0] <= t1)]
        if len(so):
            disp = math.hypot(so[-1, 2] - so[0, 2], so[-1, 3] - so[0, 3])
            P(f'  {label}: truth displacement {disp*1000:.1f} mm, yaw change {math.degrees(wrap(so[-1,4]-so[0,4])):+.3f} deg, '
              f'max |v| {np.abs(so[:,5]).max():.4f} m/s, max |w| {np.abs(so[:,6]).max():.4f} rad/s; '
              f'cmd_vel_nav msgs {len(cn)} (nonzero {int(np.sum((np.abs(cn[:,1])>1e-4)|(np.abs(cn[:,2])>1e-4))) if len(cn) else 0}), '
              f'cmd_vel msgs {len(co)} (nonzero {int(np.sum((np.abs(co[:,1])>1e-4)|(np.abs(co[:,2])>1e-4))) if len(co) else 0})')
    # the last nonzero command before / after success
    nz = cmd[(np.abs(cmd[:, 1]) > 1e-4) | (np.abs(cmd[:, 2]) > 1e-4)]
    if len(nz):
        P(f'  last nonzero /cmd_vel at {nz[-1,0]-last_goal_end:+.2f} s relative to final nav success')
    mv = odom[(np.abs(odom[:, 5]) > 0.005) | (np.abs(odom[:, 6]) > 0.01)]
    if len(mv):
        P(f'  last truth motion (|v|>5 mm/s or |w|>0.01 rad/s) at {mv[-1,0]-last_goal_end:+.2f} s relative to final nav success')
if True:
    # per-station: motion between nav success and marker confirmation / task complete
    for s in segments:
        if 'arrived' in s and 'task_complete' in s:
            so = odom[(odom[:, 0] >= s['arrived']) & (odom[:, 0] <= s['task_complete'])]
            if len(so):
                P(f'  {s["name"]}: ARRIVED->TASK COMPLETE truth displacement {1000*math.hypot(so[-1,2]-so[0,2], so[-1,3]-so[0,3]):.1f} mm, '
                  f'yaw change {math.degrees(wrap(so[-1,4]-so[0,4])):+.2f} deg, max |v| {np.abs(so[:,5]).max():.4f}')
if complete_wall:
    for a in ['navigate_to_pose', 'follow_path', 'spin', 'backup', 'wait']:
        after = [(w, [s.status for s in m.status_list]) for w, m in D[f'/{a}/_action/status'] if w > complete_wall]
        act = [w for w, ss in after if any(x in (1, 2, 3) for x in ss)]
        P(f'  {a} active-status messages after completion: {len(act)}')

(OUT / 'report.txt').write_text('\n'.join(REPORT) + '\n')

# ================================================================== plots
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(9, 12))
ext = [ox, ox + W * res, oy, oy + H * res]
ax.imshow(mapimg, cmap='gray', extent=ext, origin='upper', vmin=0, vmax=255)
gy_, gx_ = np.nonzero(gt_body)
ax.scatter(ox + gx_ * res, oy + (H - 1 - gy_) * res, s=0.3, c='tab:green', label='true geometry (LiDAR height)')
ax.plot(odom[:, 2], odom[:, 3], c='tab:blue', lw=1, label='true trajectory (Gazebo)')
ax.plot(amcl[:, 2], amcl[:, 3], c='tab:red', lw=0.8, ls='--', label='AMCL estimate')
for r in seg_rows:
    xy = r['gp'][0][1]
    ax.plot(xy[:, 0], xy[:, 1], c='orange', lw=0.8, alpha=0.8)
for n, (x, y, _) in GOALS.items():
    ax.plot(x, y, 'k*', ms=10)
for n, (x, y, _) in MARK.items():
    ax.plot(x, y, 'm+', ms=10)
ax.plot([], [], c='orange', label='first global plan per goal')
ax.legend(loc='lower left', fontsize=8)
ax.set_title('Map (grey/black), true geometry, plans and trajectories')
fig.savefig(OUT / 'trajectory_overview.png', dpi=110, bbox_inches='tight')

fig, axs = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
t0 = ms
axs[0].plot(cmdn[:, 0] - t0, cmdn[:, 1], lw=0.6, label='cmd_vel_nav v (DWB)')
axs[0].plot(odom[:, 0] - t0, odom[:, 5], lw=0.6, label='true v')
axs[0].set_ylabel('m/s'); axs[0].legend(fontsize=7)
axs[1].plot(cmdn[:, 0] - t0, cmdn[:, 2], lw=0.6, label='cmd w (DWB)')
axs[1].plot(odom[:, 0] - t0, odom[:, 6], lw=0.6, label='true w')
axs[1].set_ylabel('rad/s'); axs[1].legend(fontsize=7)
axs[2].plot(err[:, 1] - t0, np.hypot(err[:, 2], err[:, 3]), lw=0.8, label='AMCL position error (m)')
axs[2].plot(err[:, 1] - t0, np.degrees(np.abs(err[:, 4])) / 10, lw=0.8, label='|heading error| (deg/10)')
axs[2].legend(fontsize=7)
rt = np.array([(w0 + 5 - t0, (sim_at_wall(w0 + 10) - sim_at_wall(w0)) / 10) for w0 in np.arange(clk[0, 0], clk[-1, 0] - 10, 5)])
axs[3].plot(rt[:, 0], rt[:, 1], label='RTF (10 s window)')
cmt = np.array(cat_times['ctrl_miss'])
axs[3].plot(cmt - t0, np.full(len(cmt), 0.05), '|', c='r', label='controller missed rate')
for g0, g1, f in nav_goals:
    for a in axs:
        a.axvspan(g0 - t0, (g1 or me) - t0, color='grey', alpha=0.08)
axs[3].legend(fontsize=7); axs[3].set_xlabel('wall s since mission start')
fig.savefig(OUT / 'timeseries.png', dpi=100, bbox_inches='tight')
print('done')
