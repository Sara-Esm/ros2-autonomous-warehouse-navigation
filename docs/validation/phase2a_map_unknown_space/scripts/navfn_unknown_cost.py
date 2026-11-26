#!/usr/bin/env python3
"""
Measure NavFn's effective traversal cost of unknown cells with the real planner_server.

Usage: navfn_unknown_cost.py <workspace> <out_txt>

Synthetic 14 m x 14 m map: known free everywhere, with an unknown block 1.0 m deep (along the
start-goal line) and of variable width h, centered between start and goal (6 m apart). The
planner either crosses 1.0 m of unknown or detours around the block's ends. The width at which it
switches gives the effective unknown/free cost ratio: crossing cost ratio*1.0 + 5.0 free
equals the detour length.
"""
import math, os, subprocess, sys, tempfile, time
os.environ['ROS_DOMAIN_ID'] = '175'; os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np, cv2, yaml
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

ws, out = sys.argv[1], sys.argv[2]
P = yaml.safe_load(open(ws + '/src/turtlebot3/turtlebot3_navigation2/param/humble/burger_cam.yaml'))
def no_sim(d):
    if isinstance(d, dict):
        for k in d:
            if k == 'use_sim_time':
                d[k] = False
            else:
                no_sim(d[k])
no_sim(P)
P['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = True
gc = P['global_costmap']['global_costmap']['ros__parameters']
gc['plugins'] = ['static_layer', 'inflation_layer']          # no sensors in this experiment
tmp = tempfile.mkdtemp()
pf = os.path.join(tmp, 'params.yaml'); yaml.safe_dump(P, open(pf, 'w'))
res, N = 0.05, 280
lines = ['NavFn allow_unknown=true; static + inflation layers; map 14x14 m; start (-3,0) goal (+3,0); unknown block depth 1.0 m']

def run_case(h):
    img = np.full((N, N), 254, np.uint8)
    img[0, :] = img[-1, :] = img[:, 0] = img[:, -1] = 0            # boundary wall
    c = N // 2; half_w = int(round(h / 2 / res)); half_d = int(round(0.5 / res))
    img[c - half_w:c + half_w, c - half_d:c + half_d] = 205        # unknown block (rows = y, cols = x)
    pgm = os.path.join(tmp, f'm{h}.pgm'); cv2.imwrite(pgm, img)
    my = os.path.join(tmp, f'm{h}.yaml')
    open(my, 'w').write(f'image: {pgm}\nmode: trinary\nresolution: {res}\norigin: [-7.0, -7.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.01\n')
    env = dict(os.environ); procs = []
    def run(cmd):
        procs.append(subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
    run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom'])
    run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '-3.0', '--frame-id', 'odom', '--child-frame-id', 'base_link'])
    run(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p', f'yaml_filename:={my}'])
    run(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', pf])
    try:
        time.sleep(4)
        for node in ('/map_server', '/planner_server'):
            for tr in ('configure', 'activate'):
                subprocess.run(['ros2', 'lifecycle', 'set', node, tr], env=env, capture_output=True, timeout=40)
        time.sleep(2)
        n = rclpy.create_node(f'cost_probe_{int(h*100)}')
        ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose'); ac.wait_for_server(timeout_sec=20)
        def pose(x):
            ps = PoseStamped(); ps.header.frame_id = 'map'; ps.pose.position.x = x; ps.pose.orientation.w = 1.0; return ps
        g = ComputePathToPose.Goal(); g.start = pose(-3.0); g.goal = pose(3.0); g.use_start = True; g.planner_id = 'GridBased'
        f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=60)
        xy = np.array([(p.pose.position.x, p.pose.position.y) for p in rf.result().result.path.poses])
        n.destroy_node()
        if len(xy) < 2:
            raise SystemExit(f'no plan for width {h} (status {rf.result().status})')
        L = float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
        inside = (np.abs(xy[:, 0]) < 0.5) & (np.abs(xy[:, 1]) < h / 2 - 0.1)   # exclude corner-grazing cells
        return L, int(inside.sum()), float(np.abs(xy[:, 1]).max())
    finally:
        import signal
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
        left = subprocess.run(['pgrep', '-x', 'planner_server'], capture_output=True, text=True).stdout.split()
        if left:
            raise SystemExit(f'leftover planner_server processes: {left}')

rclpy.init()
rows = []
for h in (2.0, 4.0, 6.0, 7.0, 8.0, 9.0, 10.0, 12.0):
    L, inside, ymax = run_case(h)
    detour = 2 * math.hypot(3.0, h / 2)            # straight segments to the block ends
    route = 'AROUND block' if ymax >= h / 2 - 0.05 else 'THROUGH unknown'   # around requires |y| >= half-width
    rows.append((h, route, L, detour))
    lines.append(f'  block width {h:4.1f} m: plan {L:5.2f} m, poses inside block {inside:3d}, max |y| {ymax:4.2f} m -> {route}; '
                 f'geometric detour {detour:5.2f} m (extra {detour-6:5.2f} m over the 6.0 m straight line)')
rclpy.shutdown()
sw = [(a, b) for a, b in zip(rows, rows[1:]) if a[1] != b[1]]
for a, b in sw:
    lo, hi = a[3] - 6.0, b[3] - 6.0
    lines.append(f'switch between widths {a[0]} and {b[0]} m: crossing 1.0 m of unknown costs as much as '
                 f'{1+lo:.2f}-{1+hi:.2f} m of free travel -> effective unknown/free cost ratio ~{1+lo:.1f}-{1+hi:.1f} '
                 f'(header prediction 253/50 = 5.06)')
open(out, 'w').write('\n'.join(lines) + '\n'); print('\n'.join(lines))
