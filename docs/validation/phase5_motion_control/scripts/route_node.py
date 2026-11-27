#!/usr/bin/env python3
"""Fixed short route for controller experiments: spawn -> home -> loading -> home -> loading -> home.
Logs 'Navigating to <name>' / 'ARRIVED: <name>' / 'FAILED' as node 'warehouse_autonomy' so the mission
analysis scripts parse it. Goals: surveyed STATIONS_MAP (unchanged). Exit 0 only if every leg succeeds.
Bounded: 240 s wall per leg, then cancel and exit 1."""
import math, os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
sys.path.insert(0, os.path.expanduser('~/projects/warehouse_ws/src/warehouse_bringup'))
from warehouse_bringup.stations import STATIONS_MAP  # noqa: E402
ROUTES = {
    'north': [('Home Charging Dock', 'home_dock'), ('Loading Zone', 'loading_zone'), ('Home Charging Dock', 'home_dock'),
              ('Loading Zone', 'loading_zone'), ('Home Charging Dock', 'home_dock')],
    # south pinch (inventory <-> dispatch) traversed four times, four station approaches
    'south': [('Inventory Scan Station', 'inventory_scan'), ('Dispatch Zone', 'dispatch_zone'),
              ('Inventory Scan Station', 'inventory_scan'), ('Dispatch Zone', 'dispatch_zone'),
              ('Inventory Scan Station', 'inventory_scan')]}
ROUTE = ROUTES[os.environ.get('ROUTE', 'north')]
rclpy.init(); n = Node('warehouse_autonomy', parameter_overrides=[Parameter('use_sim_time', value=True)])
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose'); assert ac.wait_for_server(timeout_sec=60)
# wait_for_server only proves the client saw the server; give discovery time to match the server's
# response writer to this new client (an immediate goal was dropped once: 'Failed to send goal response').
t0 = time.monotonic()
while time.monotonic() - t0 < 3.0: rclpy.spin_once(n, timeout_sec=0.1)
rc = 0
for name, st in ROUTE:
    x, y, yaw = STATIONS_MAP[st]
    g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = x, y
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
    n.get_logger().info(f'Navigating to {name}: x={x}, y={y}, yaw={yaw} deg')
    gh = None
    for attempt in (1, 2):
        f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=30); gh = f.result()
        if gh and gh.accepted: break
        n.get_logger().warn(f'goal not acknowledged within 30 s (attempt {attempt}): {name}')
    if not gh or not gh.accepted:
        n.get_logger().error(f'FAILED (not accepted): {name}'); rc = 1; break
    rf = gh.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=240)
    if not rf.done():
        cf = gh.cancel_goal_async(); rclpy.spin_until_future_complete(n, cf, timeout_sec=10)
        n.get_logger().error(f'FAILED (timeout): {name}'); rc = 1; break
    if rf.result().status != 4:
        n.get_logger().error(f'FAILED (status {rf.result().status}): {name}'); rc = 1; break
    n.get_logger().info(f'ARRIVED: {name}')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0: rclpy.spin_once(n, timeout_sec=0.1)   # settle before next leg
n.destroy_node(); rclpy.shutdown(); sys.exit(rc)
