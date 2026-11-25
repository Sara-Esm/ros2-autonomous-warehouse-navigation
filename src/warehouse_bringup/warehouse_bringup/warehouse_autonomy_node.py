#!/usr/bin/env python3
"""Warehouse inspection mission: navigate to each station, verify its ArUco marker.

The node is a timer-driven state machine. Callbacks never sleep or spin, and
every wait is measured on the node clock, so with use_sim_time:=true all
durations are in simulation seconds.

Process exit code: 0 = all tasks succeeded, 1 = finished with failed tasks,
2 = mission halted, 130 = interrupted.
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

EXIT_SUCCESS = 0
EXIT_FAILURES = 1
EXIT_HALTED = 2
EXIT_INTERRUPTED = 130

# Mission states
WAIT_SERVER = 'WAIT_SERVER'
START_DELAY = 'START_DELAY'
NAVIGATING = 'NAVIGATING'
VERIFYING = 'VERIFYING'
TASK_HOLD = 'TASK_HOLD'
PAUSE = 'PAUSE'
FINISHED = 'FINISHED'

TICK_PERIOD_SEC = 0.1


def yaw_to_quaternion(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return qz, qw


class WarehouseAutonomy(Node):

    def __init__(self):
        super().__init__('warehouse_autonomy')

        self.start_delay = self.declare_parameter('start_delay', 10.0).value
        self.verify_timeout = self.declare_parameter('verify_timeout', 15.0).value
        self.task_hold = self.declare_parameter('task_hold', 3.0).value
        self.inter_goal_pause = self.declare_parameter('inter_goal_pause', 2.0).value

        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.last_marker_id = None
        self.marker_confirmed = False

        self.create_subscription(Int32, '/warehouse/marker_id', self.marker_id_cb, 10)

        self.waypoints = [
            {
                'name': 'Home Charging Dock',
                'task': 'Verify robot home dock and charging station',
                'x': -3.678,
                'y': 9.3,
                'yaw': 90.0,
                'marker_id': 0,
            },
            {
                'name': 'Loading Zone',
                'task': 'Verify loading area',
                'x': 1.803,
                'y': 9.3,
                'yaw': 90.0,
                'marker_id': 1,
            },
            {
                'name': 'Inventory Scan Station',
                'task': 'Verify shelf inventory using wall marker',
                'x': 1.801,
                'y': -9.0,
                'yaw': -90.0,
                'marker_id': 2,
            },
            {
                'name': 'Dispatch Zone',
                'task': 'Verify dispatch area',
                'x': -3.630,
                'y': -9.3,
                'yaw': -90.0,
                'marker_id': 3,
            },
            {
                'name': 'Return to Home Charging Dock',
                'task': 'Return robot to charging dock and close mission',
                'x': -3.678,
                'y': 9.3,
                'yaw': 90.0,
                'marker_id': 0,
            },
        ]
        self.current_index = 0
        self.successful_tasks = 0
        self.failed_tasks = 0

        self.state = WAIT_SERVER
        self.deadline = None
        self.exit_code = None

        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is false: waits are measured in wall-clock time. '
                'Run with --ros-args -p use_sim_time:=true in simulation.'
            )

        self.get_logger().info('Warehouse autonomy workflow manager started.')
        self.get_logger().info('Waiting for Nav2 action server...')

        self.timer = self.create_timer(TICK_PERIOD_SEC, self.tick)

    # ------------------------------------------------------------------ utils

    def now(self):
        return self.get_clock().now()

    def clock_valid(self):
        # With use_sim_time, the clock reads zero until the first /clock message.
        return self.now().nanoseconds > 0

    def set_deadline(self, seconds):
        self.deadline = self.now() + Duration(seconds=float(seconds))

    def deadline_passed(self):
        return self.deadline is not None and self.now() >= self.deadline

    @property
    def finished(self):
        return self.exit_code is not None

    # -------------------------------------------------------------- callbacks

    def marker_id_cb(self, msg):
        self.last_marker_id = msg.data
        if (self.state == VERIFYING and
                msg.data == self.waypoints[self.current_index]['marker_id']):
            self.marker_confirmed = True

    def tick(self):
        if self.finished or not self.clock_valid():
            return

        if self.state == WAIT_SERVER:
            if self.navigator.server_is_ready():
                self.get_logger().info(
                    f'Nav2 is ready. Starting warehouse mission in {self.start_delay:g} seconds...'
                )
                self.state = START_DELAY
                self.set_deadline(self.start_delay)

        elif self.state == START_DELAY:
            if self.deadline_passed():
                self.send_next_goal()

        elif self.state == VERIFYING:
            self.check_verification()

        elif self.state == TASK_HOLD:
            if self.deadline_passed():
                wp = self.waypoints[self.current_index]
                self.get_logger().info(f"TASK COMPLETE: {wp['name']}")
                self.successful_tasks += 1
                self.state = PAUSE
                self.set_deadline(self.inter_goal_pause)

        elif self.state == PAUSE:
            if self.deadline_passed():
                self.current_index += 1
                self.send_next_goal()

        # NAVIGATING and FINISHED advance from action callbacks only.

    # ------------------------------------------------------------- navigation

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.finish_mission()
            return

        wp = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.now().to_msg()

        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(wp['yaw'])
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg.pose = pose

        self.get_logger().info('----------------------------------------')
        self.get_logger().info(f'MISSION STEP {self.current_index + 1}/{len(self.waypoints)}')
        self.get_logger().info(
            f"Navigating to {wp['name']}: x={wp['x']}, y={wp['y']}, yaw={wp['yaw']} deg"
        )

        self.state = NAVIGATING
        self.deadline = None
        future = self.navigator.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        if self.finished:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.halt_mission(f'Goal response error: {exc}')
            return

        if not goal_handle.accepted:
            self.halt_mission('Goal rejected by Nav2')
            return

        self.get_logger().info('Goal accepted by Nav2.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        if self.finished:
            return
        wp = self.waypoints[self.current_index]
        try:
            status = future.result().status
        except Exception as exc:
            self.halt_mission(f'Navigation result error: {exc}')
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Navigation FAILED for {wp['name']} (status: {status})"
            )
            self.halt_mission(
                f"Navigation aborted at {wp['name']} "
                f'(status: {status})'
            )
            return

        self.get_logger().info(f"ARRIVED: {wp['name']}")
        self.start_verification(wp)

    # ----------------------------------------------------------- verification

    def start_verification(self, wp):
        if wp['marker_id'] is None:
            self.get_logger().info('No visual marker required for this waypoint.')
            self.on_verified(wp)
            return

        self.get_logger().info(f"Looking for ArUco marker ID {wp['marker_id']}...")
        # Only detections received after arrival count (see marker_id_cb).
        self.marker_confirmed = False
        self.state = VERIFYING
        self.set_deadline(self.verify_timeout)

    def check_verification(self):
        wp = self.waypoints[self.current_index]

        if self.marker_confirmed:
            self.get_logger().info(
                f"Correct marker detected: expected={wp['marker_id']}, "
                f'detected={self.last_marker_id}'
            )
            self.on_verified(wp)
        elif self.deadline_passed():
            self.get_logger().error(
                f"Marker verification timeout: expected={wp['marker_id']}, "
                f'last_detected={self.last_marker_id}'
            )
            self.get_logger().error(f"VISION VERIFICATION FAILED: {wp['name']}")
            self.failed_tasks += 1
            self.state = PAUSE
            self.set_deadline(self.inter_goal_pause)

    def on_verified(self, wp):
        self.get_logger().info(f"VISION VERIFIED: {wp['name']}")
        self.get_logger().info(f"TASK: {wp['task']}")
        self.state = TASK_HOLD
        self.set_deadline(self.task_hold)

    # ---------------------------------------------------------------- ending

    def log_summary(self, log_fn, unattempted):
        log_fn(f'Total mission steps: {len(self.waypoints)}')
        log_fn(f'Successful tasks: {self.successful_tasks}')
        log_fn(f'Failed tasks: {self.failed_tasks}')
        log_fn(f'Unattempted tasks: {unattempted}')
        log_fn('========================================')

    def finish_mission(self):
        log = self.get_logger()
        log.info('========================================')

        if (self.failed_tasks == 0 and
                self.successful_tasks == len(self.waypoints)):
            log.info('WAREHOUSE MISSION COMPLETE: ALL TASKS SUCCEEDED')
            self.log_summary(log.info, 0)
            self.end(EXIT_SUCCESS)
        else:
            log.error(
                f'WAREHOUSE MISSION FINISHED WITH FAILURES: '
                f'{self.failed_tasks} of {len(self.waypoints)} failed'
            )
            self.log_summary(log.info, 0)
            self.end(EXIT_FAILURES)

    def halt_mission(self, reason):
        if self.finished:
            return

        self.failed_tasks += 1
        log = self.get_logger()
        log.error('========================================')
        log.error(f'MISSION HALTED: {reason}')
        self.log_summary(
            log.error, len(self.waypoints) - self.current_index - 1
        )
        log.error('ROBOT STOP NOT VERIFIED. No further mission goals will be sent.')
        self.end(EXIT_HALTED)

    def end(self, code):
        self.state = FINISHED
        self.deadline = None
        self.exit_code = code
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseAutonomy()
    exit_code = EXIT_INTERRUPTED

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=TICK_PERIOD_SEC)
        if node.finished:
            exit_code = node.exit_code
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
