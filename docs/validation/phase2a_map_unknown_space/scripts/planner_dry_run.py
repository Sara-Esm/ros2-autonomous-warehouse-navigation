#!/usr/bin/env python3
"""
Dry-run the real Nav2 planner_server on the warehouse map (no Gazebo).

Usage: planner_dry_run.py <workspace> <map_yaml> <allow_unknown:true|false> <scan:none|spawn> <out_txt>

Runs nav2 map_server and planner_server (with the project's Nav2 parameter file, use_sim_time
forced false) on an isolated ROS domain, publishes static TFs placing the robot at the spawn pose,
and optionally a synthetic LaserScan raycast against the TRUE world geometry (what the real LiDAR
would return at spawn). Requests plans between mission points and reports whether each plan enters
cells the loaded map marks unknown.
"""
import math, os, re, subprocess, sys, tempfile, time
os.environ['ROS_DOMAIN_ID'] = '174'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np, cv2, yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

ws, map_yaml, allow_unknown, scan_mode, out = sys.argv[1:6]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup import stations as st  # noqa: E402

params_src = ws + '/src/turtlebot3/turtlebot3_navigation2/param/humble/burger_cam.yaml'
P = yaml.safe_load(open(params_src))
def no_sim(d):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == 'use_sim_time':
                d[k] = False
            else:
                no_sim(v)
no_sim(P)
P['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = (allow_unknown == 'true')
pf = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False); yaml.safe_dump(P, pf); pf.close()
gc = P['global_costmap']['global_costmap']['ros__parameters']
lines = [f'map {os.path.basename(map_yaml)}; NavFn allow_unknown={allow_unknown}; global costmap track_unknown_space={gc.get("track_unknown_space")}; '
         f'plugins={gc["plugins"]}; synthetic scan={scan_mode}']

spawn = st.world_to_map(0.0, 0.0, 0.0)
env = dict(os.environ)
procs = []
def run(cmd):
    procs.append(subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
sx, sy, syaw = spawn
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', str(sx), '--y', str(sy), '--yaw', str(math.radians(syaw)),
     '--frame-id', 'odom', '--child-frame-id', 'base_footprint'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--z', '0.010', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '-0.032', '--z', '0.171', '--frame-id', 'base_link', '--child-frame-id', 'base_scan'])
run(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={map_yaml}', '-p', 'use_sim_time:=false'])
run(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', pf.name])
try:
    time.sleep(4)
    for node in ('/map_server', '/planner_server'):
        for tr in ('configure', 'activate'):
            subprocess.run(['ros2', 'lifecycle', 'set', node, tr], env=env, capture_output=True, timeout=40)
    rclpy.init()
    n = rclpy.create_node('planner_dry_run')
    grid = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, '/map', lambda m: grid.append(m), qos)
    costmaps = []
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: costmaps.append(m), 1)
    t0 = time.time()
    while not grid and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = grid[0]; W, H = m.info.width, m.info.height; res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    loaded = np.array(m.data, dtype=np.int16).reshape(H, W)            # row 0 = lowest y
    pgm = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map.pgm', cv2.IMREAD_UNCHANGED)
    pgm205 = np.flipud(pgm) == 205                                     # same orientation as /map data
    v, c = np.unique(loaded, return_counts=True)
    lines.append('loaded /map values: ' + ', '.join(f'{int(a)}: {int(b)}' for a, b in zip(v, c)))

    if scan_mode == 'spawn':
        # true LiDAR-height geometry (world frame raster on the map grid, image row 0 = top)
        gt = np.load(ws + '/diagnostics_20261125/static/gt_lidar.npy') > 0
        GH, GW = gt.shape
        lx, ly, lyaw = st.map_to_world(*[sx + math.cos(math.radians(syaw)) * -0.032,
                                         sy + math.sin(math.radians(syaw)) * -0.032, syaw])
        ranges = []
        for k in range(360):
            a = math.radians(lyaw) + k * (6.28 / 359)
            rng = float('inf')
            for d in np.arange(0.12, 3.5, 0.01):
                wx, wy = lx + d * math.cos(a), ly + d * math.sin(a)
                ci = int(round((wx + 7.56) / 0.05)); ri = int(round(GH - 1 - (wy + 10.8) / 0.05))
                if 0 <= ri < GH and 0 <= ci < GW and gt[ri, ci]:
                    rng = d; break
            ranges.append(rng)
        scan = LaserScan(); scan.header.frame_id = 'base_scan'
        scan.angle_min = 0.0; scan.angle_max = 6.28; scan.angle_increment = 6.28 / 359
        scan.range_min = 0.12; scan.range_max = 3.5; scan.ranges = ranges
        pub = n.create_publisher(LaserScan, '/scan', 10)
        lines.append(f'synthetic scan: {sum(np.isfinite(ranges))} of 360 beams hit true geometry within 3.5 m')
        t0 = time.time()
        while time.time() - t0 < 6:
            scan.header.stamp = n.get_clock().now().to_msg(); pub.publish(scan)
            rclpy.spin_once(n, timeout_sec=0.2)
    else:
        t0 = time.time()
        while time.time() - t0 < 3:
            rclpy.spin_once(n, timeout_sec=0.2)
    costmaps.clear(); t0 = time.time()
    while not costmaps and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
    if costmaps:
        cm = np.array(costmaps[-1].data, dtype=np.int16)
        lines.append(f'global costmap: unknown(-1) {int((cm == -1).sum())}, lethal(100) {int((cm == 100).sum())}, '
                     f'inscribed(99) {int((cm == 99).sum())}, free(0) {int((cm == 0).sum())}')
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    ac.wait_for_server(timeout_sec=20)
    pts = {'spawn': spawn}
    pts.update({wp['station']: (wp['x'], wp['y'], wp['yaw']) for wp in st.WAYPOINTS})
    pairs = [('spawn', 'home_dock'), ('home_dock', 'loading_zone'), ('loading_zone', 'inventory_scan'),
             ('inventory_scan', 'dispatch_zone'), ('dispatch_zone', 'home_dock'), ('spawn', 'inventory_scan')]
    def pose(p):
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = p[0], p[1]
        ps.pose.orientation.z = math.sin(math.radians(p[2]) / 2); ps.pose.orientation.w = math.cos(math.radians(p[2]) / 2)
        return ps
    for a, b in pairs:
        goal = ComputePathToPose.Goal(); goal.goal = pose(pts[b]); goal.start = pose(pts[a])
        goal.use_start = True; goal.planner_id = 'GridBased'
        f = ac.send_goal_async(goal); rclpy.spin_until_future_complete(n, f, timeout_sec=20)
        gh = f.result()
        if gh is None or not gh.accepted:
            lines.append(f'  {a:15s} -> {b:15s}: goal not accepted'); continue
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=60)
        res_ = rf.result(); path = res_.result.path.poses if res_ else []
        if not path:
            lines.append(f'  {a:15s} -> {b:15s}: NO PLAN (status {res_.status if res_ else "timeout"})'); continue
        xy = np.array([(p.pose.position.x, p.pose.position.y) for p in path])
        L = float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
        ci = np.clip(np.floor((xy[:, 0] - ox) / res).astype(int), 0, W - 1)   # Nav2 worldToMap convention
        ri = np.clip(np.floor((xy[:, 1] - oy) / res).astype(int), 0, H - 1)
        unk = int((loaded[ri, ci] == -1).sum())
        never = int((pgm205[ri, ci]).sum())
        lines.append(f'  {a:15s} -> {b:15s}: plan {L:6.2f} m, {len(xy)} poses, on loaded-unknown cells: {unk}, '
                     f'on never-observed (PGM 205) cells: {never} ({100*never/len(xy):.0f}%)')
finally:
    for p in procs:
        p.terminate()
    for p in procs:
        try:
            p.wait(timeout=10)
        except Exception:
            p.kill()
    for name in ('planner_server', 'map_server', 'static_transform_publisher'):
        subprocess.run(['pkill', '-x', name], capture_output=True)   # exact names; -f would match callers
open(out, 'w').write('\n'.join(lines) + '\n')
print('\n'.join(lines))
