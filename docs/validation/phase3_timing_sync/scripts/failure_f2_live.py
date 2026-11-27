#!/usr/bin/env python3
"""
F2 live test node (full system running): cancel/halt, then injected persistent obstacle -> recoveries.

Usage: failure_f2_live.py <out_json>
Part A: NavigateToPose to the surveyed home dock; cancel after 20 s wall; measure cancel response time,
        final status, time until the robot is stopped (truth /odom).
Part B: resend the goal; from 5 s to 30 s publish INJECTED phantom scans (frame base_scan, 5 Hz): hits on a
        +-30 deg fan 0.6 m ahead, all other beams no-return. Then stop injecting and wait up to 150 s for the
        outcome; cancel if still running. All events are logged with wall time.
"""
import json, math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
sys.path.insert(0, os.path.expanduser('~/projects/warehouse_ws/src/warehouse_bringup'))
from warehouse_bringup.stations import STATIONS_MAP  # noqa: E402

out = sys.argv[1]
rclpy.init()
n = Node('f2_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
pub = n.create_publisher(LaserScan, '/scan', 10)
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
    x, y, yaw = STATIONS_MAP['home_dock']
    g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = x, y
    g.pose.pose.orientation.z = math.sin(math.radians(yaw) / 2); g.pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
    return f.result()
assert ac.wait_for_server(timeout_sec=60), 'navigate_to_pose not available'
spin_for(3)
# ---------------- Part A: cancel / halt
gh = goal(); ev('A: goal sent', accepted=bool(gh and gh.accepted))
spin_for(20)
t0 = time.time(); cf = gh.cancel_goal_async(); rclpy.spin_until_future_complete(n, cf, timeout_sec=15)
ev('A: cancel response', response_s=round(time.time() - t0, 3), goals_canceling=len(cf.result().goals_canceling) if cf.result() else None)
rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
ev('A: result', status=rf.result().status if rf.result() else None, after_cancel_s=round(time.time() - t0, 3))
stopped = spin_for(15, lambda: speed['v'] < 0.005 and speed['w'] < 0.01)
ev('A: robot stopped', stopped=stopped, after_cancel_s=round(time.time() - t0, 3))
spin_for(5)
# ---------------- Part B: injected persistent obstacle
gh = goal(); ev('B: goal sent', accepted=bool(gh and gh.accepted)); tb = time.time()
spin_for(5)
ev('B: injection start')
s = LaserScan(); s.header.frame_id = 'base_scan'; s.angle_min = 0.0; s.angle_max = 6.28; s.angle_increment = 6.28 / 359
s.range_min = 0.12; s.range_max = 3.5
fan = [float('inf')] * 360
for k in list(range(0, 31)) + list(range(330, 360)):
    fan[k] = 0.6
s.ranges = fan
end = time.monotonic() + 25
while time.monotonic() < end:
    s.header.stamp = n.get_clock().now().to_msg(); pub.publish(s); spin_for(0.2)
ev('B: injection stop')
rf = gh.get_result_async()
done = spin_for(150, lambda: rf.done())
if done:
    ev('B: result', status=rf.result().status, since_goal_s=round(time.time() - tb, 1))
else:
    t0 = time.time(); cf = gh.cancel_goal_async(); rclpy.spin_until_future_complete(n, cf, timeout_sec=15)
    rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
    ev('B: still running after 150 s -> canceled', status=rf.result().status if rf.result() else None, cancel_s=round(time.time() - t0, 3))
stopped = spin_for(15, lambda: speed['v'] < 0.005 and speed['w'] < 0.01)
ev('B: robot stopped', stopped=stopped)
json.dump(log, open(out, 'w'), indent=1)
n.destroy_node(); rclpy.shutdown()
