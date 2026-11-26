#!/usr/bin/env python3
"""Start nav2 map_server on a given YAML (isolated domain) and count loaded OccupancyGrid values."""
import os, subprocess, sys, time
import numpy as np
os.environ['ROS_DOMAIN_ID'] = '173'; os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
yaml = sys.argv[1]
env = dict(os.environ)
srv = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={yaml}'],
                       env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    time.sleep(3)
    for tr in ('configure', 'activate'):
        subprocess.run(['ros2', 'lifecycle', 'set', '/map_server', tr], env=env, capture_output=True, timeout=30)
    rclpy.init(); n = rclpy.create_node('map_probe'); got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.append(m), qos)
    t0 = time.time()
    while not got and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = got[0]; d = np.array(m.data, dtype=np.int16)
    v, c = np.unique(d, return_counts=True)
    print(f'{os.path.basename(yaml)}: {m.info.width}x{m.info.height} res {m.info.resolution:.3f} origin ({m.info.origin.position.x:.2f},{m.info.origin.position.y:.2f}); '
          f'loaded values ' + ', '.join(f'{int(a)}: {int(b)}' for a, b in zip(v, c)) + '  (100 occupied, 0 free, -1 unknown)')
finally:
    srv.terminate(); srv.wait(timeout=10)
