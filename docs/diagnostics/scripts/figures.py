#!/usr/bin/env python3
"""Localization-error and timing/sensor figures from the analysis outputs.

Usage: figures.py <run_dir>   (requires analyze_run.py and rigid_alignment.py outputs)
"""
import re, sys, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag2_py
run = sys.argv[1]; A = run + '/analysis/'
e = np.loadtxt(A + 'amcl_rigid_residual.csv', delimiter=',')
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
ax[0].scatter(e[:, 2], e[:, 3], s=8, label='AMCL - truth, raw (map frame taken as world)')
ax[0].scatter(e[:, 2], e[:, 4], s=8, label='after one rigid map->world transform (2.06 deg)')
ax[0].set_xlabel('true y [m] (south -> north)'); ax[0].set_ylabel('position error [m]'); ax[0].legend(fontsize=8)
ax[0].set_title('Localization error vs position along the warehouse')
ax[1].hist([e[:, 3], e[:, 4]], bins=30, label=['raw', 'after rigid fit']); ax[1].set_xlabel('position error [m]'); ax[1].legend()
fig.savefig(A + 'localization_error.png', dpi=110, bbox_inches='tight')
# timing: RTF, camera frame interval, ack timeouts
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
clk, cam, scan = [], [], []
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
while r.has_next():
    t, d, ts = r.read_next()
    if t == '/clock':
        m = deserialize_message(d, Clock); clk.append((ts * 1e-9, m.clock.sec + m.clock.nanosec * 1e-9))
    elif t == '/camera/camera_info':
        cam.append(ts * 1e-9)
    elif t == '/scan':
        scan.append(ts * 1e-9)
clk = np.array(clk); cam = np.array(cam); scan = np.array(scan)
ms = float(re.search(r'mission_start_wall: ([\d.]+)', open(run + '/meta.txt').read()).group(1))
acks = [float(m.group(1)) for l in open(run + '/launch.log') for m in [re.search(r'\[(\d{10}\.\d+)\].*acknowledge goal request', l)] if m]
fig, ax = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True)
wq = np.arange(clk[0, 0], clk[-1, 0] - 10, 5)
ax[0].plot(wq + 5 - ms, [(np.interp(x + 10, clk[:, 0], clk[:, 1]) - np.interp(x, clk[:, 0], clk[:, 1])) / 10 for x in wq])
ax[0].set_ylabel('real-time factor'); ax[0].set_ylim(0, 1)
ax[0].plot(np.array(acks) - ms, np.full(len(acks), 0.1), 'rv', label='action-ack timeout'); ax[0].legend(fontsize=8)
ax[1].plot(cam[1:] - ms, np.diff(cam), '.', ms=3); ax[1].axhline(1 / 30, c='g', ls='--', label='configured 30 Hz (nominal, sim time)'); ax[1].set_ylabel('camera frame interval [s wall]'); ax[1].legend(fontsize=8)
ax[2].plot(scan[1:] - ms, np.diff(scan), '.', ms=2); ax[2].axhline(0.2, c='g', ls='--', label='5 Hz (nominal, sim time); expected wall interval = 0.2/RTF'); ax[2].set_ylabel('LiDAR scan interval [s wall]'); ax[2].legend(fontsize=8)
ax[2].set_xlabel('wall s since mission start')
fig.savefig(A + 'timing_sensors.png', dpi=110, bbox_inches='tight')
print('done')
