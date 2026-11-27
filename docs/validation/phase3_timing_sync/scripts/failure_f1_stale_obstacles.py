#!/usr/bin/env python3
"""
F1: ClearCostmapExceptRegion(reset_distance 1.0) vs ClearEntireCostmap with INJECTED stale obstacles.

Usage: failure_f1_stale_obstacles.py <workspace> <out_txt>
Real map_server + planner_server (warehouse_map_v2, project Nav2 params, use_sim_time false), robot fixed
at the surveyed spawn estimate. A synthetic LaserScan (injected, not a physical object) marks obstacle A
0.35 m ahead (inside the preserved region) and a wall B ~1.2 m to the west (outside it); publishing then
stops, so both are stale. Reports lethal cells at A and B, static-layer integrity and a spawn->dock plan
before, after injection, and after each clear.
"""
import math, os, signal, subprocess, sys, tempfile, time
os.environ['ROS_DOMAIN_ID'] = '183'; os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np, yaml, rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap, ClearCostmapExceptRegion

ws, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup.stations import STATIONS_MAP  # noqa: E402
P = yaml.safe_load(open(ws + '/install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml'))
def no_sim(d):
    if isinstance(d, dict):
        for k in d:
            if k == 'use_sim_time': d[k] = False
            else: no_sim(d[k])
no_sim(P)
pf = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False); yaml.safe_dump(P, pf); pf.close()
RX, RY = 0.026, 0.007                      # robot (map frame), yaw 0; laser at base_link -0.032
procs = []
LOGF = open(os.path.splitext(out)[0] + '_nodes.log', 'w')
def run(cmd):
    procs.append(subprocess.Popen(cmd, stdout=LOGF, stderr=subprocess.STDOUT, start_new_session=True))
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', str(RX), '--y', str(RY), '--frame-id', 'odom', '--child-frame-id', 'base_footprint'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--z', '0.010', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '-0.032', '--z', '0.171', '--frame-id', 'base_link', '--child-frame-id', 'base_scan'])
run(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={ws}/src/warehouse_bringup/maps/warehouse_map_v2.yaml'])
run(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', pf.name])
L = ['injected obstacles: A = 0.35 m ahead of the robot (inside the preserved 1.0 m region); B = wall at map x ~ -1.2, y -0.9..+1.3 (outside)']
try:
    time.sleep(4)
    for node in ('/map_server', '/planner_server'):
        for tr in ('configure', 'activate'):
            subprocess.run(['ros2', 'lifecycle', 'set', node, tr], capture_output=True, timeout=40)
    rclpy.init(); n = rclpy.create_node('f1_probe'); grids = []
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: grids.append(m), 1)
    pub = n.create_publisher(LaserScan, '/scan', 10)
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose'); ac.wait_for_server(timeout_sec=20)
    c_all = n.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
    c_exc = n.create_client(ClearCostmapExceptRegion, '/global_costmap/clear_except_global_costmap')
    c_all.wait_for_service(timeout_sec=10); c_exc.wait_for_service(timeout_sec=10)
    def pose(x, y, yaw):
        ps = PoseStamped(); ps.header.frame_id = 'map'; ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z = math.sin(math.radians(yaw) / 2); ps.pose.orientation.w = math.cos(math.radians(yaw) / 2); return ps
    def plan():
        g = ComputePathToPose.Goal(); g.start = pose(RX, RY, 0); g.goal = pose(*STATIONS_MAP['home_dock']); g.use_start = True; g.planner_id = 'GridBased'
        f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20)
        if f.result() is None or not f.result().accepted: return 'goal NOT ACCEPTED'
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
        if rf.result() is None: return 'NO RESULT within 30 s'
        xy = [(p.pose.position.x, p.pose.position.y) for p in rf.result().result.path.poses]
        if not xy: return 'NO PLAN'
        xy = np.array(xy); return f'plan {np.sum(np.hypot(*np.diff(xy, axis=0).T)):.2f} m, min x reached {xy[:,0].min():+.2f}'
    def costmap():
        grids.clear(); t0 = time.time()
        while time.time() - t0 < 4.0: rclpy.spin_once(n, timeout_sec=0.1)
        return grids[-1]
    def cells(m):
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width); ox, oy, res = m.info.origin.position.x, m.info.origin.position.y, m.info.resolution
        def at(x, y):
            return g[int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res))]
        a = max(at(RX + 0.35 + dx, RY + dy) for dx in (0, .05, -.05) for dy in (0, .05, -.05))
        bvals = [at(-1.2 + dx, y) for y in np.arange(-0.9, 1.31, 0.1) for dx in (0, .05, -.05)]
        return f'A max cost {a}; B lethal cells {sum(v == 100 for v in bvals)} of {len(bvals)} sampled; unknown total {int((g==-1).sum())}, lethal total {int((g==100).sum())}'
    state = {'inject': False}
    def empty_scan():
        if state['inject']: return
        s = LaserScan(); s.header.frame_id = 'base_scan'; s.header.stamp = n.get_clock().now().to_msg()
        s.angle_min = 0.0; s.angle_max = 6.28; s.angle_increment = 6.28 / 359; s.range_min = 0.12; s.range_max = 3.5
        s.ranges = [float('inf')] * 360; pub.publish(s)
    n.create_timer(0.2, empty_scan)        # sensor keeps streaming but no longer sees the injected obstacles
    def inject():
        state['inject'] = True
        # beams from the laser (base_scan at robot x-0.032) to A and to points along wall B; others inf
        lx, ly = RX - 0.032, RY
        ranges = [float('inf')] * 360
        targets = [(RX + 0.35, RY)] + [(-1.2, y) for y in np.arange(-0.9, 1.31, 0.02)]
        for tx, ty in targets:
            ang = math.atan2(ty - ly, tx - lx) % (2 * math.pi); k = int(round(ang / (6.28 / 359))) % 360
            ranges[k] = min(ranges[k], math.hypot(tx - lx, ty - ly))
        s = LaserScan(); s.header.frame_id = 'base_scan'; s.angle_min = 0.0; s.angle_max = 6.28; s.angle_increment = 6.28 / 359
        s.range_min = 0.12; s.range_max = 3.5; s.ranges = ranges
        t0 = time.time()
        while time.time() - t0 < 3.0:
            s.header.stamp = n.get_clock().now().to_msg(); pub.publish(s); rclpy.spin_once(n, timeout_sec=0.2)
        state['inject'] = False
    time.sleep(3)
    L.append(f'baseline: {cells(costmap())}; {plan()}')
    for label, clear in (('ClearCostmapExceptRegion reset_distance 1.0 (implemented)', 'except'), ('ClearEntireCostmap (stock)', 'entire')):
        inject()
        t0 = time.time()
        while time.time() - t0 < 2.0: rclpy.spin_once(n, timeout_sec=0.1)   # empty scans only: A and B are now stale
        L.append(f'[{label}] after injection (stale): {cells(costmap())}; {plan()}')
        if clear == 'except':
            req = ClearCostmapExceptRegion.Request(); req.reset_distance = 1.0; fut = c_exc.call_async(req)
        else:
            fut = c_all.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
        L.append(f'[{label}] after clear: {cells(costmap())}; {plan()}')
        fut = c_all.call_async(ClearEntireCostmap.Request()); rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
        t0 = time.time()
        while time.time() - t0 < 2.0: rclpy.spin_once(n, timeout_sec=0.1)
finally:
    for p in procs:
        try: os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError: pass
    time.sleep(3)
    for p in procs:
        try: os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError: pass
open(out, 'w').write('\n'.join(L) + '\n'); print('\n'.join(L))
