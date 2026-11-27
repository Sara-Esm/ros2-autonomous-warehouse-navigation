#!/usr/bin/env python3
"""Station error at NavigateToPose acceptance vs after settling, consistent across phases (truth = /odom).

Usage: arrival_accuracy.py <label>=<run_dir> ...
acceptance: bag receipt of the navigate_to_pose status SUCCEEDED for each goal.
settled: first time after acceptance from which |v| < 2 mm/s and |w| < 0.005 rad/s hold for 1.0 s (sim stamps).
Errors are against the intended physical approach poses (world frame). Travel = truth path length from
acceptance to settled; net = straight-line displacement.
"""
import math, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
INTENDED = [(-3.678, 9.3, 90.0), (1.803, 9.3, 90.0), (1.801, -9.0, -90.0), (-3.630, -9.3, -90.0), (-3.678, 9.3, 90.0)]
NAMES = ['Home Dock', 'Loading', 'Inventory', 'Dispatch', 'Return Home']
wrap = lambda a: (a + 180) % 360 - 180
for arg in sys.argv[1:]:
    lab, run = arg.split('=', 1)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run + '/bag', storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    od, succ, seen = [], [], set()
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/odom':
            m = deserialize_message(d, ty[t]); q = m.pose.pose.orientation
            od.append((ts * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y,
                       math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))), m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif t == '/navigate_to_pose/_action/status':
            for s in deserialize_message(d, ty[t]).status_list:
                g = bytes(s.goal_info.goal_id.uuid).hex()
                if s.status == 4 and g not in seen:
                    seen.add(g); succ.append(ts * 1e-9)
    od = np.array(od); succ = sorted(succ)
    print(f'{lab}:')
    print('| Station | At acceptance: position / heading error | Speed at acceptance | Settled after | Settled: position / heading error | Travel / net displacement after acceptance |')
    print('|---|---|---|---|---|---|')
    pa, ps = [], []
    for k, w in enumerate(succ[:5]):
        ix, iy, iyaw = INTENDED[k]
        i = np.searchsorted(od[:, 0], w); a = od[min(i, len(od) - 1)]
        still = (np.abs(od[:, 5]) < 0.002) & (np.abs(od[:, 6]) < 0.005)
        j = i
        while j < len(od):
            if still[j]:
                e = j
                while e < len(od) and still[e] and od[e, 1] - od[j, 1] < 1.0: e += 1
                if e < len(od) and od[e, 1] - od[j, 1] >= 1.0: break
                j = e + 1
            else:
                j += 1
        s = od[min(j, len(od) - 1)]
        seg = od[i:j + 1]; travel = float(np.sum(np.hypot(np.diff(seg[:, 2]), np.diff(seg[:, 3])))) if len(seg) > 1 else 0.0
        ea, es = math.hypot(a[2] - ix, a[3] - iy), math.hypot(s[2] - ix, s[3] - iy); pa.append(ea); ps.append(es)
        print(f'| {NAMES[k]} | {ea:.3f} m / {wrap(a[4]-iyaw):+.1f}° | {abs(a[5])*1000:.0f} mm/s, {abs(a[6]):.3f} rad/s | {s[1]-a[1]:.2f} s sim | '
              f'{es:.3f} m / {wrap(s[4]-iyaw):+.1f}° | {travel*1000:.1f} / {1000*math.hypot(s[2]-a[2], s[3]-a[3]):.1f} mm |')
    print(f'  stations 1-4 mean at acceptance {np.mean(pa[:4]):.3f} m, settled {np.mean(ps[:4]):.3f} m; all five max acceptance {max(pa):.3f}, settled {max(ps):.3f} m')
