#!/usr/bin/env python3
"""
Collision-aware scripted coverage drive for Cartographer mapping (Phase 2b).

Follows coverage_route.ROUTE in the odometry frame using /odom, publishes /cmd_vel, and spins
slowly at marked points. It only drives the robot; the map is built by Cartographer from the real
simulated LiDAR. Safety: stops and aborts on a close LiDAR return (forward sector < STOP_FWD, any
beam < STOP_ANY), pauses when /scan or /odom go stale, aborts if stale > STALE_ABORT, and always
publishes zero velocity on exit.

Usage: coverage_drive.py <log_file>
Exit code: 0 route completed, 2 aborted (obstacle), 3 aborted (sensor loss), 130 interrupted.
"""
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coverage_route import ROUTE, WEST_AISLE_EXTENSION  # noqa: E402
EXT_START = len(ROUTE) - len(WEST_AISLE_EXTENSION)

V_MAX, V_PINCH, W_MAX, W_SPIN = 0.12, 0.10, 0.6, 0.35
V_AISLE = 0.10          # west-aisle extension speed (attempt 2)
AISLE_X = (-4.3, -3.1)
REACH, SLOW_ZONE = 0.12, 0.35
STOP_FWD, STOP_ANY, FWD_HALF_ANGLE = 0.20, 0.15, math.radians(40)
STALE_PAUSE, STALE_ABORT = 1.0, 15.0
PINCH = ((-1.70, -9.30), (0.30, -8.55))      # bounding box of the documented south pinch


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class CoverageDrive(Node):

    def __init__(self, log):
        super().__init__('coverage_drive')
        self.log = log
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pose = None
        self.odom_wall = self.scan_wall = 0.0
        self.fwd_min = self.any_min = float('inf')
        self.min_seen = float('inf')
        self.i = 1
        self.spin_left = None
        self.spin_last_yaw = None
        self.stale_since = None
        self.result = None
        self.t0 = time.monotonic()
        self.create_timer(0.05, self.step)
        if ROUTE[0][2] == 'spin':
            self.spin_left = 2 * math.pi
        self.note(f'start: {len(ROUTE)} route points')

    def note(self, text):
        line = f'{time.monotonic() - self.t0:8.1f}s {text}'
        self.log.write(line + '\n')
        self.log.flush()
        self.get_logger().info(text)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        self.odom_wall = time.monotonic()

    def on_scan(self, m):
        fwd, anym = float('inf'), float('inf')
        for k, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < m.range_min:
                continue
            a = wrap(m.angle_min + k * m.angle_increment)
            anym = min(anym, r)
            if abs(a) <= FWD_HALF_ANGLE:
                fwd = min(fwd, r)
        self.fwd_min, self.any_min = fwd, anym
        self.min_seen = min(self.min_seen, anym)
        self.scan_wall = time.monotonic()

    def stop(self):
        self.pub.publish(Twist())

    def finish(self, code, why):
        self.stop()
        self.result = code
        self.note(f'FINISH code {code}: {why}; minimum LiDAR range seen {self.min_seen:.3f} m')

    def step(self):
        if self.result is not None:
            return
        now = time.monotonic()
        if self.pose is None or now - self.odom_wall > STALE_PAUSE or now - self.scan_wall > STALE_PAUSE:
            self.stop()
            if self.stale_since is None:
                self.stale_since = now
                self.note('sensor data stale -> paused')
            elif now - self.stale_since > STALE_ABORT:
                self.finish(3, 'odometry or LiDAR lost')
            return
        if self.stale_since is not None:
            self.note('sensor data back -> resume')
            self.stale_since = None
        if self.fwd_min < STOP_FWD or self.any_min < STOP_ANY:
            self.finish(2, f'obstacle: forward {self.fwd_min:.3f} m, any {self.any_min:.3f} m at {self.pose}')
            return
        x, y, yaw = self.pose
        cmd = Twist()
        if self.spin_left is not None:                      # slow 360 deg spin
            if self.spin_last_yaw is not None:
                self.spin_left -= abs(wrap(yaw - self.spin_last_yaw))
            self.spin_last_yaw = yaw
            if self.spin_left <= 0:
                self.spin_left = self.spin_last_yaw = None
                self.note(f'spin done at ({x:+.2f},{y:+.2f})')
                if self.i >= len(ROUTE):
                    self.finish(0, 'route completed')
                    return
            else:
                cmd.angular.z = W_SPIN
                self.pub.publish(cmd)
                return
        gx, gy, action = ROUTE[self.i]
        dist = math.hypot(gx - x, gy - y)
        if dist < REACH:
            self.note(f'reached point {self.i} ({gx:+.2f},{gy:+.2f}); min LiDAR so far {self.min_seen:.2f} m')
            self.i += 1
            if action == 'spin':
                self.spin_left = 2 * math.pi
            elif self.i >= len(ROUTE):
                self.finish(0, 'route completed')
            self.stop()
            return
        err = wrap(math.atan2(gy - y, gx - x) - yaw)
        in_pinch = PINCH[0][0] <= x <= PINCH[1][0] and PINCH[0][1] <= y <= PINCH[1][1]
        in_aisle = AISLE_X[0] <= x <= AISLE_X[1] and self.i >= EXT_START
        vmax = V_PINCH if in_pinch else (V_AISLE if in_aisle else V_MAX)
        if abs(err) > 0.35:                                  # turn in place toward the next point
            cmd.angular.z = math.copysign(min(W_MAX, 0.4 + abs(err)), err) * 0.6
        else:
            cmd.linear.x = min(vmax, 0.4 * dist + 0.03) if dist < SLOW_ZONE else vmax
            cmd.angular.z = max(-W_MAX, min(W_MAX, 1.5 * err))
        cmd.linear.x = max(0.0, min(0.4, cmd.linear.x))      # hard limits
        cmd.angular.z = max(-W_MAX, min(W_MAX, cmd.angular.z))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    log = open(sys.argv[1], 'a')
    node = CoverageDrive(log)
    code = 130

    def on_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_signal)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.result
    except KeyboardInterrupt:
        node.note('interrupted')
    finally:
        for _ in range(5):
            node.stop()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    sys.exit(code)


if __name__ == '__main__':
    main()
