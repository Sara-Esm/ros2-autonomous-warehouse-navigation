#!/usr/bin/env python3
"""
Real-NavFn check of the five mission legs on a map (no Gazebo).

Usage: planner_check.py <workspace> <map_yaml_abs> <points.json> <out_txt>

points.json: {"spawn": [x, y, yaw_deg], "home_dock": [...], ...} in the map frame.
Uses the project's Nav2 parameter file as installed (use_sim_time forced false). For each plan:
length, poses on cells the loaded map marks unknown, minimum clearance to map obstacles, and
(evaluation only) minimum clearance to the true geometry via the fitted map->world transform.
"""
import json, math, os, signal, subprocess, sys, tempfile, time
os.environ['ROS_DOMAIN_ID'] = '179'; os.environ['ROS_LOCALHOST_ONLY'] = '1'
import cv2, numpy as np, yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose

ws, map_yaml, pts_file, out = sys.argv[1:5]
pts = json.load(open(pts_file))
fit = pts.pop('_eval_fit_world_from_map', None)         # [theta_deg, tx, ty] for evaluation only
P = yaml.safe_load(open(ws + '/install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml'))
def no_sim(d):
    if isinstance(d, dict):
        for k in d:
            if k == 'use_sim_time':
                d[k] = False
            else:
                no_sim(d[k])
no_sim(P)
pf = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False); yaml.safe_dump(P, pf); pf.close()
lines = [f'map {map_yaml}; NavFn allow_unknown={P["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"]}; '
         f'track_unknown_space={P["global_costmap"]["global_costmap"]["ros__parameters"]["track_unknown_space"]}']
sx, sy, syaw = pts['spawn']
procs = []
def run(cmd):
    procs.append(subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', str(sx), '--y', str(sy), '--yaw', str(math.radians(syaw)),
     '--frame-id', 'odom', '--child-frame-id', 'base_footprint'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--z', '0.010', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '-0.032', '--z', '0.171', '--frame-id', 'base_link', '--child-frame-id', 'base_scan'])
run(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={map_yaml}'])
run(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', pf.name])
try:
    time.sleep(4)
    for node in ('/map_server', '/planner_server'):
        for tr in ('configure', 'activate'):
            subprocess.run(['ros2', 'lifecycle', 'set', node, tr], capture_output=True, timeout=40)
    rclpy.init(); n = rclpy.create_node('planner_check'); grid = []
    n.create_subscription(OccupancyGrid, '/map', grid.append,
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    t0 = time.time()
    while not grid and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = grid[0]; W, H, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    loaded = np.array(m.data, dtype=np.int16).reshape(H, W)
    dmap = cv2.distanceTransform(255 - (loaded == 100).astype(np.uint8) * 255, cv2.DIST_L2, 5) * res
    gt = np.load(ws + '/diagnostics_20261125/static/gt_body.npy') > 0
    GH, GW = gt.shape; dgt = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * 0.05
    time.sleep(3)
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose'); ac.wait_for_server(timeout_sec=20)
    def pose(p):
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = p[0], p[1]
        ps.pose.orientation.z = math.sin(math.radians(p[2]) / 2); ps.pose.orientation.w = math.cos(math.radians(p[2]) / 2)
        return ps
    legs = [('spawn', 'home_dock'), ('home_dock', 'loading_zone'), ('loading_zone', 'inventory_scan'),
            ('inventory_scan', 'dispatch_zone'), ('dispatch_zone', 'home_dock')]
    for a, b in legs:
        g = ComputePathToPose.Goal(); g.start = pose(pts[a]); g.goal = pose(pts[b]); g.use_start = True; g.planner_id = 'GridBased'
        f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=60)
        path = rf.result().result.path.poses
        if not path:
            lines.append(f'  {a:15s} -> {b:15s}: NO PLAN (status {rf.result().status})'); continue
        xy = np.array([(p.pose.position.x, p.pose.position.y) for p in path])
        Lm = float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
        ci = np.clip(np.floor((xy[:, 0] - ox) / res).astype(int), 0, W - 1)
        ri = np.clip(np.floor((xy[:, 1] - oy) / res).astype(int), 0, H - 1)
        unk = int((loaded[ri, ci] == -1).sum())
        cl_map = float(dmap[ri, ci].min())
        extra = ''
        if fit:
            th = math.radians(fit[0])
            wx = math.cos(th) * xy[:, 0] - math.sin(th) * xy[:, 1] + fit[1]
            wy = math.sin(th) * xy[:, 0] + math.cos(th) * xy[:, 1] + fit[2]
            gc = np.clip(np.floor((wx + 7.56) / 0.05).astype(int), 0, GW - 1)
            gr = np.clip(GH - 1 - np.floor((wy + 10.8) / 0.05).astype(int), 0, GH - 1)
            extra = f', min clearance to true geometry {dgt[gr, gc].min():.2f} m'
        uidx = np.nonzero(loaded[ri, ci] == -1)[0]
        jmin = int(np.argmin(dmap[ri, ci]))
        detail = (f'; unknown poses at indices {uidx.tolist()} of {len(xy)}: {[tuple(np.round(xy[k], 3)) for k in uidx]}' if len(uidx) else '')
        detail += f'; min map clearance at pose {jmin} ({xy[jmin,0]:+.2f},{xy[jmin,1]:+.2f})'
        lines.append(f'  {a:15s} -> {b:15s}: plan {Lm:6.2f} m, {len(xy)} poses, poses on loaded-unknown cells {unk}, '
                     f'min clearance to map obstacles {cl_map:.2f} m{extra}{detail}')
finally:
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(3)
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
open(out, 'w').write('\n'.join(lines) + '\n'); print('\n'.join(lines))
