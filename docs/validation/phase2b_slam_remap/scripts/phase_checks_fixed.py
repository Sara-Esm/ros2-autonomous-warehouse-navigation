#!/usr/bin/env python3
"""Phase 1 supplementary checks: goal-checker estimate at arrival, process deaths, recovery triggers.

Usage: phase1_checks.py <run_dir> <workspace>
"""
import math, re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run, ws = sys.argv[1], sys.argv[2]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup import stations as st  # noqa: E402
ANSI = re.compile(r'\x1b\[[0-9;]*m')
meta = open(run + '/meta.txt').read()
ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1))
me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
yq = lambda q: math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
wrap = lambda a: (a + 180) % 360 - 180
mo, ob, bt = [], [], []
while r.has_next():
    t, d, ts = r.read_next(); w = ts * 1e-9
    if t == '/tf':
        for tr in deserialize_message(d, ty[t]).transforms:
            T = tr.transform
            row = (w, T.translation.x, T.translation.y, yq(T.rotation))
            if tr.header.frame_id == 'map':
                mo.append(row)
            elif tr.header.frame_id == 'odom':
                ob.append(row)
    elif t == '/behavior_tree_log':
        for e in deserialize_message(d, ty[t]).event_log:
            if e.current_status == 'FAILURE' and e.node_name in ('ComputePathToPose', 'FollowPath'):
                bt.append((w - ms, e.node_name))
            if e.node_name in ('Spin', 'BackUp', 'Wait') and e.current_status == 'RUNNING':
                bt.append((w - ms, e.node_name + ' START'))
mo, ob = np.array(mo), np.array(ob)
print('1) Estimated pose at ARRIVED from TF (map->odom o odom->base_footprint): what the goal checker saw')
wps = {wp['name']: wp for wp in st.WAYPOINTS}
for line in open(run + '/mission.log'):
    m = re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: ARRIVED: (.*)', line)
    if not m:
        continue
    a = float(m.group(1)); wp = wps[m.group(2).strip()]
    i = np.searchsorted(mo[:, 0], a) - 1; j = np.searchsorted(ob[:, 0], a) - 1
    x0, y0, t0 = mo[i, 1:]; x1, y1, t1 = ob[j, 1:]; th = math.radians(t0)
    ex = x0 + math.cos(th) * x1 - math.sin(th) * y1; ey = y0 + math.sin(th) * x1 + math.cos(th) * y1
    eyaw = wrap(t0 + t1)
    dxy = math.hypot(ex - wp['x'], ey - wp['y']); dyaw = wrap(eyaw - wp['yaw'])
    true_h = wrap(t1 - st.STATIONS_WORLD[wp['station']][2])      # odom yaw = world yaw
    print(f"  {wp['name']:28s} estimate-to-goal {dxy:.3f} m (tol 0.12), {dyaw:+.1f} deg (tol 8.59); "
          f"true heading error {true_h:+.1f} = estimated {dyaw:+.1f} + localization/frame {true_h - dyaw:+.1f}")
print('\n2) Process deaths relative to mission exit (post-recording window: 0..+60 s)')
lines = open(run + '/launch.log', errors='ignore').read().splitlines()
for n, line in enumerate(lines):
    if 'process has died' in line:
        stamp = next((float(mm.group(1)) for k in range(n - 1, max(0, n - 300), -1)
                      for mm in [re.search(r'\[(\d{10}\.\d+)\]', lines[k])] if mm), None)
        print(f'  last ROS timestamp before it: {stamp - me:+.1f} s rel. mission exit | {ANSI.sub("", line)[:100]}')
print('\n3) Recovery starts and preceding ack timeouts')
acks = [(float(m.group(1)) - ms, m.group(2)) for line in lines
        for m in [re.search(r'\[(\d{10}\.\d+)\].*acknowledge goal request for (\w+)', line)] if m]
for t, n in bt:
    if 'START' in n:
        print(f'  {t:6.1f} s {n}; ack timeouts in preceding 20 s: ' + str([f'{a:.1f}s {k}' for a, k in acks if -0.5 <= t - a <= 20]))
fails = sorted(set((round(t, 1), n) for t, n in bt if 'START' not in n))
want = {'ComputePathToPose': 'compute_path_to_pose', 'FollowPath': 'follow_path'}
nomatch = [f for f in fails if not any(k == want[f[1]] and -0.1 <= f[0] - a <= 1.0 for a, k in acks)]
print(f'  distinct ComputePathToPose/FollowPath failures: {len(fails)}; without a coinciding ack timeout: {nomatch}')
