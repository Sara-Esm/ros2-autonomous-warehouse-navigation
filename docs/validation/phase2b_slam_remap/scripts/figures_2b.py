#!/usr/bin/env python3
"""Phase 2b figures: original vs new map; executed path (truth via evaluation fit) and first plans on v2."""
import math, re, sys, numpy as np, cv2, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ws, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, ws + '/src/warehouse_bringup'); from warehouse_bringup.stations import STATIONS_MAP
def load(p): return cv2.imread(p, cv2.IMREAD_UNCHANGED)
old = load(ws + '/src/warehouse_bringup/maps/warehouse_map.pgm'); new = load(ws + '/src/warehouse_bringup/maps/warehouse_map_v2.pgm')
fig, ax = plt.subplots(1, 2, figsize=(13, 10))
for a, im, (ox, oy), t in ((ax[0], old, (-7.56, -10.8), 'Original map (v1.0.1, manual drive): 60.1% unknown'),
                           (ax[1], new, (-7.28, -10.7), 'New map (Phase 2b, scripted Cartographer drive): 42.4% unknown')):
    h, w = im.shape
    a.imshow(im, cmap='gray', vmin=0, vmax=255, extent=[ox, ox + w * 0.05, oy, oy + h * 0.05])
    a.set_title(t, fontsize=10); a.set_xlim(-7.8, 8.4); a.set_ylim(-11.2, 11.2); a.grid(alpha=.3)
fig.savefig(out + '/maps_original_vs_new.png', dpi=90, bbox_inches='tight')
R = ws + '/diagnostics_phase2b/run'
ms = float(re.search(r'mission_start_wall: ([\d.]+)', open(R + '/meta.txt').read()).group(1))
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=R + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}; od, plans = [], []
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/odom':
        m = deserialize_message(d, ty[t]); od.append((ts * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y))
    elif t == '/plan':
        m = deserialize_message(d, ty[t])
        if len(m.poses) > 1: plans.append((ts * 1e-9, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
od = np.array(od)
ev = [float(m.group(1)) for l in open(R + '/mission.log') for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: Navigating to', l)] if m]
th = math.radians(0.71); dx, dy = od[:, 1] + 0.009, od[:, 2] - 0.040
mx = math.cos(th) * dx + math.sin(th) * dy; my = -math.sin(th) * dx + math.cos(th) * dy
fig, ax = plt.subplots(figsize=(8, 12)); h, w = new.shape
ax.imshow(new, cmap='gray', vmin=0, vmax=255, extent=[-7.28, -7.28 + w * 0.05, -10.7, -10.7 + h * 0.05])
for s in ev:
    fp = next(xy for wt, xy in plans if wt >= s); ax.plot(fp[:, 0], fp[:, 1], c='orange', lw=1)
sel = od[:, 0] >= ev[0]; ax.plot(mx[sel], my[sel], c='tab:blue', lw=1)
for k, (x, y, yaw) in STATIONS_MAP.items(): ax.plot(x, y, 'm*', ms=12)
ax.plot([], [], c='orange', label='first global plan per leg (allow_unknown false)'); ax.plot([], [], c='tab:blue', label='executed (truth, evaluation fit)')
ax.plot([], [], 'm*', label='surveyed map-frame goals'); ax.legend(loc='lower left', fontsize=8); ax.set_title('Phase 2b mission on warehouse_map_v2')
fig.savefig(out + '/phase2b_plans_and_executed.png', dpi=90, bbox_inches='tight')
print('done')
