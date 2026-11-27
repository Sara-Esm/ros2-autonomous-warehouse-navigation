#!/usr/bin/env python3
"""Record the replayed local costmap (/costmap/costmap) of one configuration. Usage: local_unknown_probe.py <out_npz> <seconds> <cfg>"""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
out, dur, cfg = sys.argv[1], float(sys.argv[2]), sys.argv[3]
rclpy.init(); n = Node('lu_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
rec = {cfg: []}
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
def cb(k):
    def f(m):
        i = m.info; rec[k].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, i.origin.position.x, i.origin.position.y,
                                   i.width, i.height, np.array(m.data, dtype=np.int8)))
    return f
for k in rec:
    n.create_subscription(OccupancyGrid, '/costmap/costmap', cb(k), q)
end = time.monotonic() + dur
while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
np.savez_compressed(out, **{f'{k}_meta': np.array([r[:5] for r in v]) for k, v in rec.items()},
                    **{f'{k}_data': np.array([r[5] for r in v]) for k, v in rec.items() if v})
print({k: len(v) for k, v in rec.items()})
