#!/usr/bin/env python3
"""Record image / camera_info / marker / clock arrivals for a fixed wall duration.

Usage: probe.py <out_npz> <seconds> [save_images_dir]
Image subscription is RELIABLE depth 50 so every published frame is counted (not a detector-like consumer).
"""
import os, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
out, dur = sys.argv[1], float(sys.argv[2]); save = sys.argv[3] if len(sys.argv) > 3 else None
rclpy.init(); n = Node('perception_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
rec = {'img': [], 'info': [], 'marker': [], 'clock': []}
def st(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
saved = [0]
def on_img(m):
    rec['img'].append((time.time(), n.get_clock().now().nanoseconds * 1e-9, st(m.header)))
    if save and saved[0] < 20 and len(rec['img']) % 3 == 0:
        np.save(os.path.join(save, f'img_{saved[0]:02d}.npy'), np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)); saved[0] += 1
n.create_subscription(Image, '/camera/image_raw', on_img, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(CameraInfo, '/camera/camera_info', lambda m: rec['info'].append((time.time(), st(m.header))), QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(Int32, '/warehouse/marker_id', lambda m: rec['marker'].append((time.time(), n.get_clock().now().nanoseconds * 1e-9, m.data)), 50)
n.create_subscription(Clock, '/clock', lambda m: rec['clock'].append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), qos_profile_sensor_data)
if save: os.makedirs(save, exist_ok=True)
end = time.monotonic() + dur
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
np.savez(out, **{k: np.array(v, dtype=float) for k, v in rec.items()})
n.destroy_node(); rclpy.shutdown()
