#!/usr/bin/env python3
"""Capture Cartographer's published /map (raw 0-100 / -1 values) on the reload domain and save it."""
import os, sys, time
os.environ['ROS_DOMAIN_ID'] = '177'; os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np, rclpy
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node('grid_probe'); got = []
n.create_subscription(OccupancyGrid, '/map', got.append, 10)
t0 = time.time()
while not got and time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.2)
m = got[-1]; g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
np.savez(sys.argv[1], grid=g, ox=m.info.origin.position.x, oy=m.info.origin.position.y, res=m.info.resolution)
print('grid', g.shape, 'origin', m.info.origin.position.x, m.info.origin.position.y)
