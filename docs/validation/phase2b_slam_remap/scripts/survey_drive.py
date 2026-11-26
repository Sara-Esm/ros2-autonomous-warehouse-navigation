#!/usr/bin/env python3
"""
Station survey for Phase 2b (localization session on the new map).

Drives the robot (odometry control, as an operator would position it) to each intended physical
approach pose in front of its marker, then records the localization estimate in the MAP frame:
three cycles of +-DITHER yaw dithers (each > AMCL update_min_a) followed by a 1.5 s average of TF
map->base_footprint and the latest /amcl_pose covariance. Ground truth (/odom = Gazebo world pose)
is logged alongside for independent validation only; the surveyed goal is the localization mean.

Usage: survey_drive.py <out_json> <log_file>
Exit: 0 survey complete, 2 obstacle stop, 3 sensor/TF loss, 130 interrupted.
"""
import json, math, os, signal, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

# Intended physical approach poses (world/odometry frame) and safe access routes (from the coverage route).
PLAN = [
    ('spawn', [], (0.0, 0.0, 0.0)),
    ('home_dock', [(-1.5, 0.4), (-3.6, 0.6), (-3.7, 3.0), (-3.7, 6.5), (-3.678, 8.6)], (-3.678, 9.3, 90.0)),
    ('loading_zone', [(-3.678, 8.6), (-3.4, 6.9), (-1.5, 6.85), (0.5, 7.1), (1.6, 7.6), (1.803, 8.6)], (1.803, 9.3, 90.0)),
    ('inventory_scan', [(1.803, 8.6), (1.75, 7.0), (1.7, 4.5), (1.7, 1.0), (1.7, -3.0), (1.75, -6.0), (1.801, -8.3)], (1.801, -9.0, -90.0)),
    ('dispatch_zone', [(1.801, -8.4), (0.2, -8.75), (-0.95, -8.85), (-1.6, -9.15), (-3.63, -8.7)], (-3.63, -9.3, -90.0)),
    ('return_spawn', [(-3.63, -8.7), (-3.8, -6.0), (-3.8, -2.0), (-3.6, 0.6), (-1.5, 0.4)], None),
]
V, V_FINE, W_MAX = 0.12, 0.05, 0.5
REACH, FINE_TOL, YAW_TOL = 0.10, 0.015, math.radians(0.4)
DITHER, CYCLES = 0.30, 3
STOP_FWD, STOP_ANY, FWD_HALF = 0.20, 0.15, math.radians(40)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Survey(Node):

    def __init__(self, log):
        super().__init__('station_survey', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.log = log
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.tf = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.pose = self.amcl = None
        self.odom_t = self.scan_t = 0.0
        self.fwd = self.anyr = float('inf'); self.min_seen = float('inf')
        self.amcl_updates = 0

    def note(self, s):
        self.log.write(f'{time.strftime("%H:%M:%S")} {s}\n'); self.log.flush(); self.get_logger().info(s)

    def on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        self.odom_t = time.monotonic()

    def on_scan(self, m):
        f = a = float('inf')
        for k, r in enumerate(m.ranges):
            if math.isfinite(r) and r >= m.range_min:
                a = min(a, r)
                if abs(wrap(m.angle_min + k * m.angle_increment)) <= FWD_HALF:
                    f = min(f, r)
        self.fwd, self.anyr = f, a; self.min_seen = min(self.min_seen, a); self.scan_t = time.monotonic()

    def on_amcl(self, m):
        self.amcl = m; self.amcl_updates += 1

    def spin_for(self, sec, cmd=None):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.check()
            if cmd is not None:
                self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    def check(self):
        now = time.monotonic()
        if self.pose is None or now - self.odom_t > 2.0 or now - self.scan_t > 2.0:
            self.stop(); raise SystemExit(3)
        if self.fwd < STOP_FWD or self.anyr < STOP_ANY:
            self.stop(); self.note(f'OBSTACLE STOP fwd {self.fwd:.3f} any {self.anyr:.3f} at {self.pose}'); raise SystemExit(2)

    def stop(self):
        self.pub.publish(Twist())

    def wait_ready(self):
        t0 = time.monotonic()
        while (self.pose is None or self.scan_t == 0.0) and time.monotonic() - t0 < 60:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.check()

    def goto(self, x, y, tol=REACH, vmax=V):
        while True:
            self.check(); rclpy.spin_once(self, timeout_sec=0.05)
            px, py, yaw = self.pose
            d = math.hypot(x - px, y - py)
            if d < tol:
                self.stop(); return
            err = wrap(math.atan2(y - py, x - px) - yaw)
            c = Twist()
            if abs(err) > 0.3:
                c.angular.z = math.copysign(min(W_MAX, 0.25 + abs(err) * 0.5), err)
            else:
                c.linear.x = min(vmax, 0.5 * d + 0.02)
                c.angular.z = max(-W_MAX, min(W_MAX, 1.5 * err))
            self.pub.publish(c)

    def turn_to(self, yaw_t):
        while True:
            self.check(); rclpy.spin_once(self, timeout_sec=0.05)
            err = wrap(yaw_t - self.pose[2])
            if abs(err) < YAW_TOL:
                self.stop(); self.spin_for(0.6, Twist()); return
            c = Twist(); c.angular.z = math.copysign(max(0.06, min(0.4, 1.2 * abs(err))), err)
            self.pub.publish(c)

    def sample(self):
        xs, ys, cs, ss = [], [], [], []
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf.lookup_transform('map', 'base_footprint', Time())
            except Exception:
                continue
            xs.append(t.transform.translation.x); ys.append(t.transform.translation.y)
            a = yaw_of(t.transform.rotation); cs.append(math.cos(a)); ss.append(math.sin(a))
        if not xs:
            raise SystemExit(3)
        cov = list(self.amcl.pose.covariance) if self.amcl else [float('nan')] * 36
        return {'map_x': sum(xs) / len(xs), 'map_y': sum(ys) / len(ys),
                'map_yaw_deg': math.degrees(math.atan2(sum(ss), sum(cs))),
                'amcl_sigma_x': math.sqrt(cov[0]), 'amcl_sigma_y': math.sqrt(cov[7]),
                'amcl_sigma_yaw_deg': math.degrees(math.sqrt(cov[35])),
                'truth_x': self.pose[0], 'truth_y': self.pose[1], 'truth_yaw_deg': math.degrees(self.pose[2]),
                'amcl_updates_so_far': self.amcl_updates}

    def survey_here(self, name, target):
        tx, ty, tyaw = target
        self.goto(tx, ty, tol=FINE_TOL, vmax=V_FINE)
        self.turn_to(math.radians(tyaw))
        samples = []
        for k in range(CYCLES):
            for d in (DITHER, 0.0, -DITHER, 0.0):
                self.turn_to(math.radians(tyaw) + d)
            self.spin_for(2.0, Twist())
            samples.append(self.sample())
            self.note(f'{name} sample {k+1}: map ({samples[-1]["map_x"]:+.4f},{samples[-1]["map_y"]:+.4f},{samples[-1]["map_yaw_deg"]:+.2f}) '
                      f'truth ({samples[-1]["truth_x"]:+.4f},{samples[-1]["truth_y"]:+.4f},{samples[-1]["truth_yaw_deg"]:+.2f}) '
                      f'sigma ({samples[-1]["amcl_sigma_x"]:.3f},{samples[-1]["amcl_sigma_y"]:.3f},{samples[-1]["amcl_sigma_yaw_deg"]:.2f} deg)')
        return samples


def main():
    out, logf = sys.argv[1], open(sys.argv[2], 'a')
    rclpy.init(); n = Survey(logf); code = 130; results = {}

    def on_sig(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sig)
    try:
        n.wait_ready()
        for name, route, target in PLAN:
            for x, y in route:
                n.goto(x, y)
            if target is None:
                n.goto(0.0, 0.0)
                n.turn_to(0.0)
                continue
            results[name] = {'intended_world': target, 'samples': n.survey_here(name, target)}
            json.dump(results, open(out, 'w'), indent=1)
        code = 0
        n.note(f'SURVEY COMPLETE; min LiDAR range {n.min_seen:.3f} m; AMCL updates {n.amcl_updates}')
    except SystemExit as e:
        code = int(e.code) if e.code is not None else 1
        n.note(f'SURVEY ABORTED code {code}')
    except KeyboardInterrupt:
        n.note('interrupted')
    finally:
        for _ in range(5):
            n.stop(); time.sleep(0.05)
        json.dump(results, open(out, 'w'), indent=1)
        n.destroy_node(); rclpy.shutdown(); logf.close()
    sys.exit(code)


if __name__ == '__main__':
    main()
