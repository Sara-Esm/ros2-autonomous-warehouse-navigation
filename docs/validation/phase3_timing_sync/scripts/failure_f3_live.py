#!/usr/bin/env python3
"""
F3 live test node: unreachable goal (inside the central block; injected fault) -> planning failures and
BT recoveries. F3a: cancel 45 s after sending (during the recovery cycle). F3b: resend and let the BT
abort by itself (bounded 150 s; canceled if still running). Logs wall-time events and the robot stop.

Usage: failure_f3_live.py <out_json>
"""
import json, math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
out = sys.argv[1]
rclpy.init(); n = Node('f3_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
speed = {'v': 0.0, 'w': 0.0}
n.create_subscription(Odometry, '/odom', lambda m: speed.update(v=abs(m.twist.twist.linear.x), w=abs(m.twist.twist.angular.z)), 10)
log = {'events': []}
def ev(s, **k):
    log['events'].append(dict(t_wall=time.time(), what=s, **k)); print(f'{time.strftime("%H:%M:%S")} {s} {k}', flush=True)
def spin_for(sec, cond=None):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if cond and cond(): return True
    return False
def goal():
    g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = -1.6, 2.35; g.pose.pose.orientation.w = 1.0
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=30); return f.result()
assert ac.wait_for_server(timeout_sec=60)
spin_for(3)
gh = goal(); ev('F3a: unreachable goal sent', accepted=bool(gh and gh.accepted))
spin_for(45)
t0 = time.time(); cf = gh.cancel_goal_async(); rclpy.spin_until_future_complete(n, cf, timeout_sec=15)
ev('F3a: cancel response', response_s=round(time.time() - t0, 3))
rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
ev('F3a: result', status=rf.result().status if rf.result() else None, after_cancel_s=round(time.time() - t0, 3))
ev('F3a: robot stopped', stopped=spin_for(15, lambda: speed['v'] < 0.005 and speed['w'] < 0.01), after_cancel_s=round(time.time() - t0, 3))
spin_for(5)
gh = goal(); tb = time.time(); ev('F3b: unreachable goal sent', accepted=bool(gh and gh.accepted))
rf = gh.get_result_async()
if spin_for(150, lambda: rf.done()):
    ev('F3b: result', status=rf.result().status, since_goal_s=round(time.time() - tb, 1))
else:
    cf = gh.cancel_goal_async(); rclpy.spin_until_future_complete(n, cf, timeout_sec=15); rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
    ev('F3b: still running after 150 s -> canceled', status=rf.result().status if rf.result() else None)
ev('F3b: robot stopped', stopped=spin_for(15, lambda: speed['v'] < 0.005 and speed['w'] < 0.01))
json.dump(log, open(out, 'w'), indent=1); n.destroy_node(); rclpy.shutdown()
