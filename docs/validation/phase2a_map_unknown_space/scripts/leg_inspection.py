#!/usr/bin/env python3
"""Inspect plan evolution, unknown traversal locations and minimum-clearance points per leg.

Usage: leg_inspection.py <phase2a_run> <phase1_run> <workspace> <out_dir>
"""
import bisect, math, re, sys
import cv2, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r2, r1, ws, out = sys.argv[1:5]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup import stations as st  # noqa: E402
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map.pgm', cv2.IMREAD_UNCHANGED)
H, W = img.shape; RES, OX, OY = 0.05, -7.56, -10.8
gt = np.load(ws + '/diagnostics_20261125/static/gt_body.npy') > 0
dgt = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * RES
def idx(x, y):
    c = np.clip(np.floor((np.asarray(x) - OX) / RES).astype(int), 0, W - 1)
    return H - 1 - np.clip(np.floor((np.asarray(y) - OY) / RES).astype(int), 0, H - 1), c
def load(run):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in rd.get_all_topics_and_types()}
    plans, od, cms = [], [], []
    while rd.has_next():
        t, d, ts = rd.read_next(); w = ts * 1e-9
        if t == '/plan':
            m = deserialize_message(d, ty[t])
            if len(m.poses) > 1:
                plans.append((w, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
        elif t == '/odom':
            m = deserialize_message(d, ty[t]); od.append((w, m.pose.pose.position.x, m.pose.pose.position.y))
        elif t == '/global_costmap/costmap':
            m = deserialize_message(d, ty[t])
            cms.append((w, m.info.origin.position.x, m.info.origin.position.y,
                        np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)))
    ev = [(float(m.group(1)), m.group(2), m.group(3)) for l in open(run + '/mission.log')
          for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED)(.*)', l)] if m]
    legs = [(n.split(':')[0].strip(), a, b) for (a, k, n), (b, _, _) in zip(ev, ev[1:]) if k == 'Navigating to']
    return plans, np.array(od), cms, legs
P2, O2, C2, L2 = load(r2); P1, O1, _, L1 = load(r1)
plen = lambda xy: float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
lines = []
Pr = lambda s='': (lines.append(s), print(s))
for k in (0, 2, 4):
    name, a, b = L2[k]
    lp = [(w, xy) for w, xy in P2 if a <= w <= b]
    lens = np.array([plen(xy) for _, xy in lp])
    remaining = []
    for w, xy in lp:
        i = np.searchsorted(O2[:, 0], w); x, y = O2[min(i, len(O2) - 1), 1:3]
        mx, my, _ = st.world_to_map(x, y, 0.0)
        remaining.append(math.hypot(xy[-1, 0] - mx, xy[-1, 1] - my))
    # successive-plan route changes (one-sided Hausdorff of new plan vs previous)
    def dev(p, q):
        sub = q[::max(1, len(q) // 50)]
        return max(np.min(np.hypot(p[:, 0] - s[0], p[:, 1] - s[1])) for s in sub)
    ch = [(lp[i][0] - a, dev(lp[i - 1][1], lp[i][1])) for i in range(1, len(lp))]
    big = [(round(t, 1), round(d, 2)) for t, d in ch if d > 0.5]
    Pr(f'## {name}: {len(lp)} plans; first {lens[0]:.2f} m; route changes > 0.5 m between consecutive plans: {big}')
    o = O2[(O2[:, 0] >= a) & (O2[:, 0] <= b)]
    gr, gc = idx(o[:, 1], o[:, 2]); clr = dgt[gr, gc]; j = int(np.argmin(clr))
    Pr(f'   min clearance to true geometry {clr[j]:.2f} m at world ({o[j,1]:+.2f},{o[j,2]:+.2f}), {o[j,0]-a:.1f} s into the leg; '
       f'time with clearance < 0.35 m: {np.sum(np.diff(o[:,0])[clr[1:] < 0.35]):.1f} s wall')
    n1, a1, b1 = L1[k]
    o1 = O1[(O1[:, 0] >= a1) & (O1[:, 0] <= b1)]
    gr1, gc1 = idx(o1[:, 1], o1[:, 2]); c1 = dgt[gr1, gc1]; j1 = int(np.argmin(c1))
    Pr(f'   Phase 1 same leg: min clearance {c1[j1]:.2f} m at world ({o1[j1,1]:+.2f},{o1[j1,2]:+.2f})')
# figure: map frame, unknown shaded, Phase 1 vs Phase 2a executed (truth converted to map frame)
vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
vis[img == 205] = (205, 205, 205); vis[img == 254] = (255, 255, 255)
def draw(od, col):
    pts = [idx(*st.world_to_map(x, y, 0.0)[:2]) for _, x, y in od[::5]]
    for (r0, c0), (r1_, c1_) in zip(pts, pts[1:]):
        cv2.line(vis, (int(c0), int(r0)), (int(c1_), int(r1_)), col, 1)
big = cv2.resize(vis, (W * 3, H * 3), interpolation=cv2.INTER_NEAREST)
vis = big
def draw3(od, col):
    pts = [idx(*st.world_to_map(x, y, 0.0)[:2]) for _, x, y in od[::3]]
    for (r0, c0), (r1_, c1_) in zip(pts, pts[1:]):
        cv2.line(vis, (int(c0 * 3 + 1), int(r0 * 3 + 1)), (int(c1_ * 3 + 1), int(r1_ * 3 + 1)), col, 2)
draw3(O1[(O1[:, 0] >= L1[0][1]) & (O1[:, 0] <= L1[-1][2])], (0, 140, 255))      # Phase 1 orange
draw3(O2[(O2[:, 0] >= L2[0][1]) & (O2[:, 0] <= L2[-1][2])], (200, 60, 0))       # Phase 2a blue
for wp in st.WAYPOINTS:
    rr, cc = idx(wp['x'], wp['y']); cv2.circle(vis, (int(cc * 3 + 1), int(rr * 3 + 1)), 6, (180, 0, 180), -1)
cv2.putText(vis, 'grey: never observed (unknown)  orange: Phase 1 executed  blue: Phase 2a executed', (10, H * 3 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
cv2.imwrite(out + '/executed_paths_phase1_vs_phase2a.png', vis)
open(out + '/leg_inspection.txt', 'w').write('\n'.join(lines) + '\n')
