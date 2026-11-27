#!/usr/bin/env python3
"""Return-home plans vs the closest preceding recorded global costmap (Phase 3).

Usage: plan_vs_live_costmap.py <run_dir> <workspace>
For every /plan on the return leg: snapshot age; live costmap value at each plan pose (OccupancyGrid
scale: -1 unknown, 0 free, 1-98 inflation cost, 99 inscribed, 100 lethal), split by the static map class.
Plans and costmap are both in the map frame (costmap origin checked against the map origin).
"""
import bisect, math, re, sys
import cv2, numpy as np, rosbag2_py, yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run, ws = sys.argv[1], sys.argv[2]
my = yaml.safe_load(open(ws + '/src/warehouse_bringup/maps/warehouse_map_v2.yaml'))
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map_v2.pgm', cv2.IMREAD_UNCHANGED); H, W = img.shape
ox, oy, res = my['origin'][0], my['origin'][1], my['resolution']
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
plans, cms = [], []
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/plan':
        m = deserialize_message(d, ty[t])
        if len(m.poses) > 1: plans.append((ts * 1e-9, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
    elif t == '/global_costmap/costmap':
        m = deserialize_message(d, ty[t])
        cms.append((ts * 1e-9, m.info.origin.position.x, m.info.origin.position.y, m.info.width, m.info.height,
                    np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)))
ev = [float(m.group(1)) for l in open(run + '/mission.log') for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to Return|ARRIVED: Return)', l)] if m]
a, b = ev[0], ev[1]
cw = [c[0] for c in cms]
print(f'costmap origin ({cms[0][1]:.3f},{cms[0][2]:.3f}) size {cms[0][3]}x{cms[0][4]} vs map origin ({ox},{oy}) size {W}x{H}')
ages, cls = [], {'static free': [], 'static unknown': [], 'static occupied': []}
for w, xy in plans:
    if not a <= w <= b: continue
    i = bisect.bisect_right(cw, w) - 1
    if i < 0: continue
    _, cox, coy, cwid, chei, g = cms[i]; ages.append(w - cw[i])
    cx = np.clip(np.floor((xy[:, 0] - cox) / res).astype(int), 0, cwid - 1); cy = np.clip(np.floor((xy[:, 1] - coy) / res).astype(int), 0, chei - 1)
    live = g[cy, cx]
    mx = np.clip(np.floor((xy[:, 0] - ox) / res).astype(int), 0, W - 1); mr = H - 1 - np.clip(np.floor((xy[:, 1] - oy) / res).astype(int), 0, H - 1)
    st = img[mr, mx]
    for name, val in (('static free', 254), ('static unknown', 205), ('static occupied', 0)):
        cls[name].extend(live[st == val].tolist())
ages = np.array(ages)
print(f'return-leg plans: {len(ages)}; snapshot age (plan time - preceding costmap): median {np.median(ages):.2f} s, max {ages.max():.2f} s (wall)')
for name, v in cls.items():
    v = np.array(v)
    if len(v) == 0: print(f'  {name}: 0 poses'); continue
    print(f'  {name}: {len(v)} plan poses -> live unknown(-1) {np.sum(v==-1)}, free(0) {np.sum(v==0)}, inflation 1-98 {np.sum((v>0)&(v<99))}, '
          f'inscribed(99) {np.sum(v==99)}, lethal(100) {np.sum(v==100)}')
