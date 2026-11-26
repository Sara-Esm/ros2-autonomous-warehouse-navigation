#!/usr/bin/env python3
"""Load a map YAML with nav2 map_server (isolated domain) and compare every loaded cell with the PGM class."""
import os, subprocess, sys, time
os.environ['ROS_DOMAIN_ID'] = '178'; os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np, yaml, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
ypath = sys.argv[1]
meta = yaml.safe_load(open(ypath))
pgm_path = os.path.join(os.path.dirname(ypath), meta['image'])
data = open(pgm_path, 'rb').read(); parts = data.split(maxsplit=4)
w, h = int(parts[1]), int(parts[2]); img = np.frombuffer(parts[4][:w * h], np.uint8).reshape(h, w)
expected = np.flipud(np.where(img == 0, 100, np.where(img == 254, 0, np.where(img == 205, -1, 99))))
srv = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={ypath}'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
try:
    time.sleep(3)
    for tr in ('configure', 'activate'):
        subprocess.run(['ros2', 'lifecycle', 'set', '/map_server', tr], capture_output=True, timeout=40)
    rclpy.init(); n = rclpy.create_node('percell_probe'); got = []
    n.create_subscription(OccupancyGrid, '/map', got.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    t0 = time.time()
    while not got and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = got[0]; g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    v, c = np.unique(g, return_counts=True)
    mism = int((g != expected).sum())
    print(f'{os.path.basename(ypath)} free_thresh={meta["free_thresh"]}: {m.info.width}x{m.info.height}, origin ({m.info.origin.position.x:.6f},{m.info.origin.position.y:.6f}); '
          f'loaded ' + ', '.join(f'{int(a)}: {int(b)}' for a, b in zip(v, c)) + f'; cells differing from PGM class: {mism}')
finally:
    import signal; os.killpg(srv.pid, signal.SIGINT); srv.wait(timeout=10)
