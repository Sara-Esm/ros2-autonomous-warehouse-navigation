#!/usr/bin/env python3
"""Perception metrics for a recorded mission (identical method for Phase 3 and Phase 4 runs).

Usage: perception_metrics.py <label>=<run_dir> [...]
Window: mission_start_wall .. mission_end_wall.
- camera_info rate (sim, wall): published once per rendered camera frame (verified on the bench), used as
  the delivered image rate because raw images were not recorded (same in both runs).
- camera_info age: sim clock at bag receipt - header stamp (sim ms).
- detector processed-frame rate: /warehouse/marker_detected (one message per processed image).
- marker message rate and IDs: /warehouse/marker_id.
- time to verification per station: 'Looking for ArUco marker' -> 'Correct marker detected' (wall s, from
  mission.log), and whether each verification succeeded.
"""
import re, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for arg in sys.argv[1:]:
    label, run = arg.split('=', 1)
    meta = open(run + '/meta.txt').read()
    ms = float(re.search(r'mission_start_wall: ([\d.]+)', meta).group(1)); me = float(re.search(r'mission_end_wall: ([\d.]+)', meta).group(1))
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    clk, info, det, mk = [], [], [], []
    while r.has_next():
        t, d, ts = r.read_next(); w = ts * 1e-9
        if not ms <= w <= me: continue
        if t == '/clock':
            m = deserialize_message(d, ty[t]); clk.append((w, m.clock.sec + m.clock.nanosec * 1e-9))
        elif t == '/camera/camera_info':
            m = deserialize_message(d, ty[t]); info.append((w, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
        elif t == '/warehouse/marker_detected':
            det.append((w, deserialize_message(d, ty[t]).data))
        elif t == '/warehouse/marker_id':
            mk.append((w, deserialize_message(d, ty[t]).data))
    clk = np.array(clk); sim = lambda x: np.interp(x, clk[:, 0], clk[:, 1])
    info = np.array(info); det = np.array(det, dtype=float)
    simspan = sim(me) - sim(ms); wallspan = me - ms
    age = (sim(info[:, 0]) - info[:, 1]) * 1e3
    gaps = np.diff(info[:, 1])
    ids = sorted(set(int(v) for _, v in mk))
    print(f'{label}: mission {wallspan:.0f} s wall, {simspan:.0f} s sim, RTF {simspan/wallspan:.3f}')
    print(f'  camera_info: {len(info)/simspan:.2f} Hz sim, {len(info)/wallspan:.2f} Hz wall; frame interval (stamp) p50 {np.median(gaps)*1e3:.0f} ms, '
          f'p95 {np.percentile(gaps,95)*1e3:.0f} ms, max {gaps.max()*1e3:.0f} ms; age p50 {np.median(age):.0f} ms sim, p95 {np.percentile(age,95):.0f}, max {age.max():.0f}')
    print(f'  detector processed frames: {len(det)/simspan:.2f} Hz sim, {len(det)/wallspan:.2f} Hz wall ({len(det)} frames, '
          f'{int(det[:,1].sum()) if len(det) else 0} with a marker); marker_id messages {len(mk)}; ids seen {ids}')
    log = open(run + '/mission.log', errors='ignore').read().splitlines()
    look = None; rows = []
    for l in log:
        m = re.search(r'\[(\d{10}\.\d+)\].*Looking for ArUco marker ID (\d+)', l)
        if m: look = (float(m.group(1)), int(m.group(2))); continue
        m = re.search(r'\[(\d{10}\.\d+)\].*(Correct marker detected: expected=(\d+), detected=(\d+)|Marker verification timeout)', l)
        if m and look:
            ok = 'Correct' in m.group(2); rows.append((look[1], ok, float(m.group(1)) - look[0])); look = None
    print('  verification: ' + ', '.join(f'ID {i} {"OK" if ok else "FAIL"} {dt:.2f} s' for i, ok, dt in rows)
          + f' | {sum(ok for _, ok, _ in rows)}/{len(rows)} verified, mean {np.mean([d for *_, d in rows]):.2f} s wall')
