#!/usr/bin/env python3
"""
F4 live test node: cancel NavigateToPose WHILE a recovery behaviour is executing (halt path).

Usage: failure_f4_live.py <out_json>
Sends the unreachable goal (inside the central block; injected fault). When spin/backup/wait reports a
goal EXECUTING, waits 1.5 s and cancels the navigation. Records the behaviour goal's final status and
timing, the navigation result, the robot stop, and any behaviour still active 5 s later. Repeats for up
to two recovery types (bounded).
"""
import json, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_action_status_default
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
out = sys.argv[1]
rclpy.init(); n = Node('f4_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
speed = {'v': 0.0, 'w': 0.0}
n.create_subscription(Odometry, '/odom', lambda m: speed.update(v=abs(m.twist.twist.linear.x), w=abs(m.twist.twist.angular.z)), 10)
beh = {}          # action -> {goal_id: (status, wall_time)}
def on_status(a, m):
    for s in m.status_list:
        beh.setdefault(a, {})[bytes(s.goal_info.goal_id.uuid).hex()] = (s.status, time.time())
for a in ('spin', 'backup', 'wait'):
    n.create_subscription(GoalStatusArray, f'/{a}/_action/status', lambda m, a=a: on_status(a, m), qos_profile_action_status_default)
log = {'events': []}
def ev(s, **k):
    log['events'].append(dict(t_wall=time.time(), what=s, **k)); print(f'{time.strftime("%H:%M:%S")} {s} {k}', flush=True)
def spin_for(sec, cond=None):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if cond and cond(): return True
    return False
def executing():
    for a, g in beh.items():
        for gid, (st, _) in g.items():
            if st == 2: return a, gid
    return None
assert ac.wait_for_server(timeout_sec=60); spin_for(3)
done_types = set()
for attempt in range(2):
    g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = -1.6, 2.35; g.pose.pose.orientation.w = 1.0
    beh.clear(); f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=30); gh = f.result()
    ev(f'attempt {attempt}: unreachable goal sent', accepted=bool(gh and gh.accepted))
    target = None
    if not spin_for(60, lambda: (executing() is not None and executing()[0] not in done_types)):
        ev(f'attempt {attempt}: no new recovery behaviour executing within 60 s'); continue
    a, gid = executing(); done_types.add(a); ev(f'attempt {attempt}: {a} executing; cancel navigation in 1.5 s')
    spin_for(1.5)
    t0 = time.time(); cf = gh.cancel_goal_async(); rclpy.spin_until_future_complete(n, cf, timeout_sec=15)
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
    ev(f'attempt {attempt}: navigation result', status=rf.result().status if rf.result() else None, after_cancel_s=round(time.time() - t0, 3))
    spin_for(15, lambda: beh[a][gid][0] in (4, 5, 6))
    st, tw = beh[a][gid]
    ev(f'attempt {attempt}: {a} goal final', status=st, after_cancel_s=round(tw - t0, 3))
    ev(f'attempt {attempt}: robot stopped', stopped=spin_for(15, lambda: speed['v'] < 0.005 and speed['w'] < 0.01), after_cancel_s=round(time.time() - t0, 3))
    spin_for(5)
    still = [(x, gg) for x, gs in beh.items() for gg, (s, _) in gs.items() if s in (1, 2, 3)]
    ev(f'attempt {attempt}: behaviour goals still active 5 s later', count=len(still))
json.dump(log, open(out, 'w'), indent=1); n.destroy_node(); rclpy.shutdown()
