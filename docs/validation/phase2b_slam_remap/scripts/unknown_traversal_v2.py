#!/usr/bin/env python3
"""
Unknown-space traversal, clearance and leg timing for one recorded mission.

Usage: unknown_traversal.py <run_dir> <workspace> <label> [out_txt]

Per leg (mission log 'Navigating to' -> 'ARRIVED'):
- Planned: the first /plan of the leg and the mean over all /plan messages of the leg: length and
  share/length on STATIC-unknown cells (PGM value 205 = never observed while mapping). If the bag has
  /global_costmap/costmap, also the share on LIVE-unknown cells (still -1 in the latest global
  costmap at plan time, i.e. not yet observed/cleared by the LiDAR).
- Executed: Gazebo truth (/odom = world pose) converted to the map frame with the EVALUATION fit for v2 (run via eval_wrapper.py); originally the Phase 1
  calibration; distance and sim time while the robot centre is on static-unknown cells, and (if
  available) on live-unknown cells.
- Clearance: distance from the truth centre to the true LiDAR-height geometry (diagnostics raster,
  world frame) and to map obstacles; +-0.025 m raster quantization.
Cell indexing follows Nav2 worldToMap: floor((x - origin) / res); image row = H - 1 - my.
"""
import bisect, math, re, sys
from collections import defaultdict
import cv2, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

run, ws, label = sys.argv[1], sys.argv[2], sys.argv[3]
out = sys.argv[4] if len(sys.argv) > 4 else run + '/analysis/unknown_traversal.txt'
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup import stations as st  # noqa: E402
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map_v2.pgm', cv2.IMREAD_UNCHANGED)
H, W = img.shape; RES, OX, OY = 0.05, -7.28, -10.7   
static_unknown = img == 205
dmap = cv2.distanceTransform(255 - (img == 0).astype(np.uint8) * 255, cv2.DIST_L2, 5) * RES
gt = np.load(ws + '/diagnostics_20261125/static/gt_body.npy') > 0
dgt = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * RES


def idx(x, y):
    c = np.clip(np.floor((np.asarray(x) - OX) / RES).astype(int), 0, W - 1)
    my = np.clip(np.floor((np.asarray(y) - OY) / RES).astype(int), 0, H - 1)
    return H - 1 - my, c


r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
plans, odom, cms, clk = [], [], [], []
while r.has_next():
    t, d, ts = r.read_next(); w = ts * 1e-9
    if t == '/plan':
        m = deserialize_message(d, ty[t])
        if len(m.poses) > 1:
            plans.append((w, np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])))
    elif t == '/odom':
        m = deserialize_message(d, ty[t])
        odom.append((w, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y))
    elif t == '/global_costmap/costmap':
        m = deserialize_message(d, ty[t])
        cms.append((w, m.info.width, m.info.height, m.info.origin.position.x, m.info.origin.position.y,
                    np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)))
    elif t == '/clock':
        m = deserialize_message(d, ty[t]); clk.append((w, m.clock.sec + m.clock.nanosec * 1e-9))
odom = np.array(odom); clk = np.array(clk)
cm_w = [c[0] for c in cms]


def live_unknown_at(w, x, y):
    """True where the latest global costmap before wall time w still marks the cell unknown."""
    i = bisect.bisect_right(cm_w, w) - 1
    if i < 0:
        return None
    _, cw, ch, cox, coy, grid = cms[i]
    cx = np.clip(np.floor((np.asarray(x) - cox) / RES).astype(int), 0, cw - 1)
    cy = np.clip(np.floor((np.asarray(y) - coy) / RES).astype(int), 0, ch - 1)
    return grid[cy, cx] == -1


ev = [(float(m.group(1)), m.group(2), m.group(3)) for l in open(run + '/mission.log')
      for m in [re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED|MISSION HALTED)(.*)', l)] if m]
legs = []
for (w0, k0, n0), (w1, k1, _) in zip(ev, ev[1:]):
    if k0 == 'Navigating to':
        legs.append((n0.split(':')[0].strip(), w0, w1))
meta = open(run + '/meta.txt').read()
ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1))
me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
launch = open(run + '/launch.log', errors='ignore').read().splitlines()
acks = [float(m.group(1)) for l in launch for m in [re.search(r'\[(\d{10}\.\d+)\].*acknowledge goal request', l)] if m]
L = []
P = lambda s='': (L.append(s), print(s))
P(f'# {label}: unknown-space traversal, clearance, leg timing ({run})')
P(f'live global costmap recorded: {"yes, " + str(len(cms)) + " messages" if cms else "no (static-unknown only)"}')
P('| Leg | Wall s | Sim s | First plan m | First plan on static-unknown | Mean over plans (static) | Mean over plans (live) | '
  'Executed m | Executed on static-unknown (m / sim s) | Executed on live-unknown (m) | Min clearance true geometry | p5 | Min clearance map | Ack timeouts |')
P('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
tot = defaultdict(float)
sim = lambda w: float(np.interp(w, clk[:, 0], clk[:, 1]))
for name, w0, w1 in legs:
    lp = [(w, xy) for w, xy in plans if w0 <= w <= w1]
    def plen(xy):
        return float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
    def on_static(xy):
        rr, cc = idx(xy[:, 0], xy[:, 1]); return static_unknown[rr, cc]
    fp = lp[0][1]
    fseg = np.hypot(*np.diff(fp, axis=0).T); fsu = on_static(fp)[1:]
    first_unk_m = float(np.sum(fseg[fsu]))
    mean_static = np.mean([on_static(xy).mean() for _, xy in lp])
    lives = [live_unknown_at(w, xy[:, 0], xy[:, 1]) for w, xy in lp]
    mean_live = np.mean([lv.mean() for lv in lives if lv is not None]) if cms else float('nan')
    sel = (odom[:, 0] >= w0) & (odom[:, 0] <= w1)
    o = odom[sel]
    mx, my, _ = zip(*[st.world_to_map(x, y, 0.0) for x, y in o[:, 2:4]])
    mx, my = np.array(mx), np.array(my)
    seg = np.hypot(np.diff(mx), np.diff(my)); dts = np.diff(o[:, 1])
    rr, cc = idx(mx, my); su = static_unknown[rr, cc][1:]
    ex_static_m, ex_static_t = float(seg[su].sum()), float(dts[su].sum())
    if cms:
        lu = np.array([bool(live_unknown_at(w, x, y)) for w, x, y in zip(o[1:, 0], mx[1:], my[1:])])
        ex_live_m = float(seg[lu].sum())
    else:
        ex_live_m = float('nan')
    GH, GW = gt.shape                                 # GT raster has its own origin (-7.56, -10.8)
    gc = np.clip(np.floor((o[:, 2] + 7.56) / 0.05).astype(int), 0, GW - 1)
    gr = GH - 1 - np.clip(np.floor((o[:, 3] + 10.8) / 0.05).astype(int), 0, GH - 1)
    clr = dgt[gr, gc]; clm = dmap[rr, cc]
    na = sum(w0 <= a <= w1 for a in acks)
    P(f'| {name} | {w1-w0:.1f} | {sim(w1)-sim(w0):.1f} | {plen(fp):.2f} | {first_unk_m:.2f} m ({100*fsu.mean():.0f}%) | '
      f'{100*mean_static:.0f}% | {100*mean_live:.0f}% | {seg.sum():.2f} | {ex_static_m:.2f} / {ex_static_t:.1f} | {ex_live_m:.2f} | '
      f'{clr.min():.2f} | {np.percentile(clr,5):.2f} | {clm.min():.2f} | {na} |')
    for k, v in (('wall', w1 - w0), ('sim', sim(w1) - sim(w0)), ('exec', seg.sum()), ('ex_static', ex_static_m),
                 ('ex_static_t', ex_static_t), ('ex_live', ex_live_m), ('first_unk', first_unk_m), ('first', plen(fp))):
        tot[k] += v
P(f'| **Total** | {tot["wall"]:.1f} | {tot["sim"]:.1f} | {tot["first"]:.2f} | {tot["first_unk"]:.2f} m | | | {tot["exec"]:.2f} | '
  f'{tot["ex_static"]:.2f} / {tot["ex_static_t"]:.1f} | {tot["ex_live"]:.2f} | | | | {sum(ms <= a <= me for a in acks)} |')
mins = (me - ms) / 60
P(f'\nack timeouts: {len(acks)} in launch log, {sum(ms <= a <= me for a in acks)} during the mission '
  f'({sum(ms <= a <= me for a in acks)/mins:.2f} per wall minute, {sum(ms <= a <= me for a in acks)/max(tot["sim"]/60,1e-9):.2f} per sim minute of navigation)')
P(f'executed share on static-unknown cells: {100*tot["ex_static"]/tot["exec"]:.1f}% of {tot["exec"]:.1f} m')
open(out, 'w').write('\n'.join(L) + '\n')
