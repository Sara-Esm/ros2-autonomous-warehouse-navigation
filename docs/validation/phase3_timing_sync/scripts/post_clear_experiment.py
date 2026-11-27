#!/usr/bin/env python3
"""
Does planning right after a global-costmap clear fail with allow_unknown=false? (real planner_server)

Usage: post_clear_experiment.py <workspace> <out_txt>
map_server + planner_server on warehouse_map_v2 with the project's Nav2 parameters (use_sim_time false),
static TFs at the spawn pose. For each clear service and delay, clear the global costmap, wait, request a
plan spawn -> home dock, and record success. Also samples the published global costmap right after a clear.
"""
import math, os, signal, subprocess, sys, tempfile, time
os.environ['ROS_DOMAIN_ID'] = '181'; os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np, yaml, rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap, ClearCostmapExceptRegion

ws, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup.stations import STATIONS_MAP  # noqa: E402
P = yaml.safe_load(open(ws + '/install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml'))
def no_sim(d):
    if isinstance(d, dict):
        for k in d:
            if k == 'use_sim_time':
                d[k] = False
            else:
                no_sim(d[k])
no_sim(P)
if os.environ.get('ALLOW_UNKNOWN'):
    P['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = os.environ['ALLOW_UNKNOWN'] == 'true'
pf = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False); yaml.safe_dump(P, pf); pf.close()
gc = P['global_costmap']['global_costmap']['ros__parameters']
L = [f'allow_unknown={P["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"]}, '
     f'global costmap update_frequency={gc["update_frequency"]} Hz, publish_frequency={gc["publish_frequency"]} Hz']
procs = []
def run(cmd):
    procs.append(subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True))
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '0.026', '--y', '0.007', '--frame-id', 'odom', '--child-frame-id', 'base_footprint'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--z', '0.010', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'])
run(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '-0.032', '--z', '0.171', '--frame-id', 'base_link', '--child-frame-id', 'base_scan'])
run(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '-p',
     f'yaml_filename:={ws}/src/warehouse_bringup/maps/warehouse_map_v2.yaml'])
run(['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', pf.name])
try:
    time.sleep(4)
    for node in ('/map_server', '/planner_server'):
        for tr in ('configure', 'activate'):
            subprocess.run(['ros2', 'lifecycle', 'set', node, tr], capture_output=True, timeout=40)
    rclpy.init(); n = rclpy.create_node('post_clear_probe')
    grids = []
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: grids.append((time.monotonic(), m)), 1)
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose'); ac.wait_for_server(timeout_sec=20)
    clr_all = n.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
    clr_exc = n.create_client(ClearCostmapExceptRegion, '/global_costmap/clear_except_global_costmap')
    clr_all.wait_for_service(timeout_sec=10); clr_exc.wait_for_service(timeout_sec=10)
    time.sleep(3)
    def pose(x, y, yaw):
        ps = PoseStamped(); ps.header.frame_id = 'map'; ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z = math.sin(math.radians(yaw) / 2); ps.pose.orientation.w = math.cos(math.radians(yaw) / 2); return ps
    def plan():
        g = ComputePathToPose.Goal(); g.start = pose(0.026, 0.007, 0.0); g.goal = pose(*STATIONS_MAP['home_dock'])
        g.use_start = True; g.planner_id = 'GridBased'
        t0 = time.monotonic(); f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
        return len(rf.result().result.path.poses) > 0, time.monotonic() - t0
    def clear(kind):
        if kind == 'entire':
            fut = clr_all.call_async(ClearEntireCostmap.Request())
        else:
            req = ClearCostmapExceptRegion.Request(); req.reset_distance = 3.0; fut = clr_exc.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    ok, dt = plan(); L.append(f'baseline plan: {"OK" if ok else "FAILED"} ({dt*1e3:.0f} ms)')
    MODE = os.environ.get('RACE_ONLY')
    for kind in (() if MODE else ('entire', 'except_region(reset_distance 3.0 m)')):
        for delay in (0.0, 0.2, 0.5, 1.0, 2.0):
            res = []
            for rep in range(3):
                time.sleep(2.5)                                   # let the costmap settle between trials
                clear('entire' if kind == 'entire' else 'except')
                time.sleep(delay)
                ok, dt = plan(); res.append(ok)
            L.append(f'  clear {kind:36s} then plan after {delay:.1f} s: {sum(res)}/3 plans succeeded')
    # race test: clear while a plan is being computed (plan request sent first, clear immediately after)
    def plan_async():
        g = ComputePathToPose.Goal(); g.start = pose(0.026, 0.007, 0.0); g.goal = pose(*STATIONS_MAP['home_dock'])
        g.use_start = True; g.planner_id = 'GridBased'
        return ac.send_goal_async(g)
    for kind in ('entire', 'except'):
        for offset in ((0.0,) if MODE else (0.0, 0.005, 0.015)):
            fails = 0
            N = int(MODE) if MODE else 20
            for rep in range(N):
                time.sleep(1.2)
                f = plan_async(); time.sleep(offset)
                if kind == 'entire':
                    cf = clr_all.call_async(ClearEntireCostmap.Request())
                else:
                    req = ClearCostmapExceptRegion.Request(); req.reset_distance = 3.0; cf = clr_exc.call_async(req)
                rclpy.spin_until_future_complete(n, f, timeout_sec=20)
                rf = f.result().get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
                rclpy.spin_until_future_complete(n, cf, timeout_sec=10)
                fails += len(rf.result().result.path.poses) == 0
            L.append(f'  RACE: plan request, then {kind:6s} clear {offset*1e3:4.0f} ms later: {fails}/{N} plans FAILED')
    # costmap snapshot right after an entire clear
    grids.clear(); time.sleep(2.5); clear('entire'); t_c = time.monotonic()
    while time.monotonic() - t_c < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    for tm, m in grids:
        d = np.array(m.data, dtype=np.int8)
        L.append(f'  costmap published {tm - t_c:+.2f} s after entire clear: unknown {int((d==-1).sum())} of {d.size}, free {int((d==0).sum())}')
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
open(out, 'w').write('\n'.join(L) + '\n'); print('\n'.join(L))
