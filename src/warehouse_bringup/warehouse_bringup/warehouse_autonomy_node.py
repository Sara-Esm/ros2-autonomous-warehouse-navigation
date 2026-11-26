#!/usr/bin/env python3
"""
Warehouse inspection mission: navigate to each station, verify its ArUco marker.

Design
------
* One state machine, driven by a timer on the steady (wall) clock, so the
  watchdogs keep working when the simulation clock stalls. Mission waits
  (start delay, verification window, holds, navigation timeout) are measured
  on the node clock, i.e. in simulation time with use_sim_time:=true.
  Communication watchdogs (Nav2 startup, goal acceptance, cancel
  confirmation, readiness, /clock stall) are measured in wall time.
* Before every goal (first attempt and retries), a readiness gate requires:
  the NavigateToPose server is discovered, bt_navigator reports ACTIVE, no
  Nav2 action server (navigate_to_pose, follow_path, spin, ...) reports an
  active goal, and odometry shows the robot stationary. One goal at a time;
  responses from stale or abandoned goals are ignored, and late-accepted
  goals are canceled.
* A navigation that exceeds nav_timeout is canceled, and the mission halts.
  An aborted navigation is retried up to nav_retries times, and only via the
  readiness gate. The gate also requires that follow_path was observed active
  during the failed attempt, so a retry is never sent while an orphaned
  controller goal may still be driving. If no such activity is seen and every
  monitored action reports idle, the gate stops waiting after
  retry_evidence_grace and halts instead of holding until gate_timeout.
* After a halt the node observes odometry and Nav2 action status for a short
  window and reports whether a stop was verified. This is passive monitoring
  only: the node never publishes velocity commands, and it is not a safety
  controller.

Process exit code: 0 = all tasks succeeded, 1 = finished with failed tasks,
2 = mission halted, 3 = mission not started (startup failure),
130 = interrupted.
"""

import math
import signal
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.signals import SignalHandlerOptions

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

from warehouse_bringup.nav2_supervision import ActionActivityTracker, MotionTracker
# Map-frame station goals, derived from world-frame approach poses (see stations.py).
from warehouse_bringup.stations import WAYPOINTS

EXIT_SUCCESS = 0
EXIT_FAILURES = 1
EXIT_HALTED = 2
EXIT_NOT_STARTED = 3
EXIT_INTERRUPTED = 130

# Mission states
STARTUP = 'STARTUP'
START_DELAY = 'START_DELAY'
GATE = 'GATE'
AWAIT_RESPONSE = 'AWAIT_RESPONSE'
NAVIGATING = 'NAVIGATING'
CANCELING = 'CANCELING'
VERIFYING = 'VERIFYING'
TASK_HOLD = 'TASK_HOLD'
PAUSE = 'PAUSE'
RETRY_PAUSE = 'RETRY_PAUSE'
HALT_OBSERVE = 'HALT_OBSERVE'
FINISHED = 'FINISHED'

TICK_PERIOD_SEC = 0.1
NS = 1_000_000_000

DEFAULT_MONITORED_ACTIONS = [
    'navigate_to_pose', 'follow_path', 'compute_path_to_pose',
    'spin', 'backup', 'wait', 'drive_on_heading', 'assisted_teleop',
]


def yaw_to_quaternion(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class WarehouseAutonomy(Node):

    def __init__(self, **node_kwargs):
        super().__init__('warehouse_autonomy', **node_kwargs)

        p = self.declare_parameter
        # Mission waits: simulation time when use_sim_time is true.
        self.start_delay = p('start_delay', 10.0).value
        self.verify_timeout = p('verify_timeout', 15.0).value
        self.task_hold = p('task_hold', 3.0).value
        self.inter_goal_pause = p('inter_goal_pause', 2.0).value
        self.retry_pause = p('retry_pause', 3.0).value
        self.nav_timeout = p('nav_timeout', 300.0).value
        self.stop_hold = p('stop_hold', 1.0).value
        self.nav_retries = p('nav_retries', 1).value
        # Watchdogs: wall-clock time, independent of /clock.
        self.startup_timeout = p('startup_timeout', 180.0).value
        self.gate_timeout = p('gate_timeout', 120.0).value
        self.goal_response_timeout = p('goal_response_timeout', 20.0).value
        self.cancel_timeout = p('cancel_timeout', 10.0).value
        self.clock_stall_timeout = p('clock_stall_timeout', 15.0).value
        self.retry_evidence_grace = p('retry_evidence_grace', 5.0).value
        self.halt_observe_timeout = p('halt_observe_timeout', 15.0).value
        self.odom_max_age = p('odom_max_age', 2.0).value
        # Supervision inputs.
        self.check_lifecycle = p('check_lifecycle', True).value
        self.lifecycle_node = p('lifecycle_node', 'bt_navigator').value
        self.monitored_actions = p('monitored_actions', DEFAULT_MONITORED_ACTIONS).value
        odom_topic = p('odom_topic', '/odom').value
        linear_tol = p('stop_linear_tolerance', 0.01).value
        angular_tol = p('stop_angular_tolerance', 0.02).value

        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)

        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle_client = self.create_client(
            GetState, f'/{self.lifecycle_node}/get_state')

        self.activity = ActionActivityTracker()
        self.motion = MotionTracker(linear_tol, angular_tol, self.odom_max_age)

        self.create_subscription(Int32, '/warehouse/marker_id', self.marker_id_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.attempt_publishers = {}
        for action in self.monitored_actions:
            self.create_subscription(
                GoalStatusArray, f'/{action}/_action/status',
                lambda msg, name=action: self.activity.update(name, msg.status_list),
                qos_profile_action_status_default)

        self.waypoints = [dict(wp) for wp in WAYPOINTS]
        self.current_index = 0
        self.successful_tasks = 0
        self.failed_tasks = 0
        self.nav_attempt = 0

        self.last_marker_id = None
        self.marker_confirmed = False
        self.confirmed_marker_id = None

        # Goal bookkeeping: one goal at a time, identified by goal_seq.
        self.goal_seq = 0
        self.goal_handle = None
        self.goal_active = False
        self.goal_pending = False
        self.cancel_requested = False
        self.require_controller_evidence = False
        self.step_dispatched = False
        self.goals_sent = 0

        self.lifecycle_state = None
        self.lifecycle_label = 'unknown'
        self.lifecycle_result_wall = 0
        self.lifecycle_query_wall = 0
        self.lifecycle_future = None

        self.state = STARTUP
        self.sim_deadline = None
        self.wall_deadline = self.wall_ns() + int(self.startup_timeout * NS)
        self.state_entered_wall = self.wall_ns()
        self.last_wait_log_wall = 0
        self.last_sim_ns = None
        self.last_sim_change_wall = None
        self.exit_code = None
        # Outcome, for reporting and tests.
        self.unattempted_tasks = None
        self.halt_reason = None
        self.stop_verified = None

        if not self.use_sim_time:
            self.get_logger().warn(
                'use_sim_time is false: mission waits use wall-clock time. '
                'Run with --ros-args -p use_sim_time:=true in simulation.')

        self.get_logger().info('Warehouse autonomy workflow manager started.')
        self.get_logger().info('Waiting for Nav2 action server...')

        self.timer = self.create_timer(TICK_PERIOD_SEC, self.tick, clock=self.steady)

    # ------------------------------------------------------------- clocks

    def wall_ns(self):
        return self.steady.now().nanoseconds

    def sim_now(self):
        return self.get_clock().now()

    def clock_valid(self):
        # With use_sim_time, the clock reads zero until the first /clock message.
        return self.sim_now().nanoseconds > 0

    def set_sim_deadline(self, seconds):
        self.sim_deadline = self.sim_now() + Duration(seconds=float(seconds))

    def sim_deadline_passed(self):
        return self.sim_deadline is not None and self.sim_now() >= self.sim_deadline

    def set_wall_deadline(self, seconds):
        self.wall_deadline = self.wall_ns() + int(seconds * NS)

    def wall_deadline_passed(self):
        return self.wall_deadline is not None and self.wall_ns() >= self.wall_deadline

    def enter(self, state):
        self.state = state
        self.state_entered_wall = self.wall_ns()
        self.last_wait_log_wall = self.state_entered_wall

    @property
    def finished(self):
        return self.exit_code is not None

    @property
    def wp(self):
        return self.waypoints[self.current_index]

    # ---------------------------------------------------------- callbacks

    def marker_id_cb(self, msg):
        self.last_marker_id = msg.data
        if self.state == VERIFYING and msg.data == self.wp['marker_id']:
            # Latch the confirming ID: later detections must not change what
            # the success log reports.
            self.confirmed_marker_id = msg.data
            self.marker_confirmed = True

    def odom_cb(self, msg):
        twist = msg.twist.twist
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * NS + stamp.nanosec
        if stamp_ns <= 0 or abs(self.sim_now().nanoseconds - stamp_ns) > self.odom_max_age * NS:
            self.motion.reset_hold()
            return
        self.motion.update(twist.linear.x, twist.linear.y, twist.angular.z,
                           stamp_ns, self.wall_ns())

    def poll_lifecycle(self):
        if not self.check_lifecycle:
            return
        now = self.wall_ns()
        if self.lifecycle_future is not None:
            if now - self.lifecycle_query_wall > 2 * NS:
                self.lifecycle_future = None  # unanswered; ask again
            return
        if now - self.lifecycle_query_wall < NS:
            return
        self.lifecycle_query_wall = now
        if not self.lifecycle_client.service_is_ready():
            self.lifecycle_state = None
            self.lifecycle_label = 'service unavailable'
            self.lifecycle_result_wall = now
            return
        self.lifecycle_future = self.lifecycle_client.call_async(GetState.Request())
        self.lifecycle_future.add_done_callback(self.lifecycle_cb)

    def lifecycle_cb(self, future):
        if future is not self.lifecycle_future:
            return
        self.lifecycle_future = None
        try:
            state = future.result().current_state
        except Exception as exc:
            self.lifecycle_state = None
            self.lifecycle_label = f'query error: {exc}'
        else:
            self.lifecycle_state = state.id
            self.lifecycle_label = state.label
        self.lifecycle_result_wall = self.wall_ns()

    def lifecycle_active(self, since_wall=0):
        if not self.check_lifecycle:
            return True
        return (self.lifecycle_state == LifecycleState.PRIMARY_STATE_ACTIVE and
                self.lifecycle_result_wall >= since_wall and
                self.wall_ns() - self.lifecycle_result_wall < 3 * NS)

    # --------------------------------------------------------------- tick

    def tick(self):
        if self.finished:
            return
        self.poll_lifecycle()
        if self.check_clock_stall():
            return

        handler = {
            STARTUP: self.tick_startup,
            START_DELAY: self.tick_start_delay,
            GATE: self.tick_gate,
            AWAIT_RESPONSE: self.tick_await_response,
            NAVIGATING: self.tick_navigating,
            CANCELING: self.tick_canceling,
            VERIFYING: self.tick_verifying,
            TASK_HOLD: self.tick_task_hold,
            PAUSE: self.tick_pause,
            RETRY_PAUSE: self.tick_retry_pause,
            HALT_OBSERVE: self.tick_halt_observe,
        }.get(self.state)
        if handler:
            handler()

    def check_clock_stall(self):
        """Halt if simulation time stops advancing during the mission."""
        if not self.use_sim_time or self.state in (STARTUP, HALT_OBSERVE, FINISHED):
            return False
        sim = self.sim_now().nanoseconds
        wall = self.wall_ns()
        if self.last_sim_ns is not None and sim < self.last_sim_ns:
            self.halt('Simulation clock moved backwards')
            return True
        if sim != self.last_sim_ns or self.last_sim_change_wall is None:
            self.last_sim_ns = sim
            self.last_sim_change_wall = wall
            return False
        if wall - self.last_sim_change_wall > self.clock_stall_timeout * NS:
            self.halt(
                f'Simulation clock stalled: /clock did not advance for '
                f'{self.clock_stall_timeout:g} s (wall)')
            return True
        return False

    def log_waiting(self, what):
        now = self.wall_ns()
        if now - self.last_wait_log_wall >= 10 * NS:
            self.last_wait_log_wall = now
            self.get_logger().warn(f'Still waiting ({self.state}): {what}')

    def tick_startup(self):
        unmet = []
        if not self.clock_valid():
            unmet.append('no /clock received')
        if not self.navigator.server_is_ready():
            unmet.append('navigate_to_pose action server not available')
        if not self.lifecycle_active():
            unmet.append(f'{self.lifecycle_node} not ACTIVE ({self.lifecycle_label})')
        if not unmet:
            self.get_logger().info(
                f'Nav2 is ready. Starting warehouse mission in {self.start_delay:g} seconds...')
            self.enter(START_DELAY)
            self.set_sim_deadline(self.start_delay)
        elif self.wall_deadline_passed():
            self.not_started(
                f'Nav2 not ready after {self.startup_timeout:g} s (wall): ' + '; '.join(unmet))
        else:
            self.log_waiting('; '.join(unmet))

    def tick_start_delay(self):
        if self.sim_deadline_passed():
            self.enter_gate(retry=False)

    # --------------------------------------------------------------- gate

    def enter_gate(self, retry):
        self.gate_retry = retry
        self.enter(GATE)
        self.set_wall_deadline(self.gate_timeout)

    def status_publishers(self):
        return {
            name: tuple(sorted(bytes(info.endpoint_gid) for info in
                               self.get_publishers_info_by_topic(f'/{name}/_action/status')))
            for name in self.monitored_actions
        }

    def publisher_problems(self, publishers):
        """
        List monitored status publishers that are missing or have changed.

        Used by both the readiness gate and halt observation: it says nothing
        about an earlier navigation attempt.
        """
        reasons = []
        for name, identities in publishers.items():
            if len(identities) != 1:
                reasons.append(f'expected one {name} status publisher, found {len(identities)}')
            elif self.attempt_publishers and identities != self.attempt_publishers.get(name):
                reasons.append(f'{name} status publisher changed during mission')
        return reasons

    def retry_evidence_missing(self):
        """Report whether an abort retry still lacks observed controller activity."""
        return (self.require_controller_evidence and
                not self.activity.saw_activity('follow_path'))

    def gate_problems(self, publishers):
        """Readiness problems: publisher identity plus abort-retry evidence."""
        reasons = self.publisher_problems(publishers)
        if self.retry_evidence_missing():
            reasons.append('controller activity for this attempt was not observed')
        return reasons

    def gate_unmet(self):
        """List the readiness conditions that are not yet satisfied."""
        unmet = []
        wall = self.wall_ns()
        if not self.navigator.server_is_ready():
            unmet.append('navigate_to_pose action server not available')
        if not self.lifecycle_active(since_wall=self.state_entered_wall):
            unmet.append(f'{self.lifecycle_node} not confirmed ACTIVE ({self.lifecycle_label})')
        publishers = self.status_publishers()
        unmet.extend(self.gate_problems(publishers))
        active = self.activity.active_actions()
        if active or self.goal_active:
            names = dict(active)
            if self.goal_active:
                names.setdefault('navigate_to_pose', 1)
            unmet.append('Nav2 goals still active: ' +
                         ', '.join(f'{k}({v})' for k, v in sorted(names.items())))
        if not self.motion.fresh(wall, self.odom_max_age):
            unmet.append('no recent odometry')
        elif not self.motion.stationary_for_sim(self.sim_now().nanoseconds, self.stop_hold):
            unmet.append(f'robot not stationary for {self.stop_hold:g} s')
        if not unmet:
            self.attempt_publishers = publishers
        return unmet

    def tick_gate(self):
        unmet = self.gate_unmet()
        if not unmet:
            self.send_goal()
        elif (self.retry_evidence_missing() and self.activity.idle() and
                self.wall_ns() - self.state_entered_wall >
                self.retry_evidence_grace * NS):
            self.halt(
                'Controller activity was not observed within the retry evidence '
                'grace period; retry withheld '
                f'(waited {self.retry_evidence_grace:g} s wall): ' + '; '.join(unmet))
        elif self.wall_deadline_passed():
            self.halt(
                f"Readiness check failed before navigating to {self.wp['name']} "
                f'(waited {self.gate_timeout:g} s wall): ' + '; '.join(unmet))
        else:
            self.log_waiting('; '.join(unmet))

    # --------------------------------------------------------- navigation

    def send_goal(self):
        wp = self.wp
        retry = self.gate_retry
        self.nav_attempt = self.nav_attempt + 1 if retry else 1

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.sim_now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        qz, qw = yaw_to_quaternion(wp['yaw'])
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        if retry:
            self.get_logger().warn(
                f"RETRYING navigation to {wp['name']} "
                f'(attempt {self.nav_attempt}/{self.nav_retries + 1})')
        else:
            self.get_logger().info('----------------------------------------')
            self.get_logger().info(
                f'MISSION STEP {self.current_index + 1}/{len(self.waypoints)}')
        self.get_logger().info(
            f"Navigating to {wp['name']}: x={wp['x']}, y={wp['y']}, yaw={wp['yaw']} deg")

        self.goal_seq += 1
        self.goals_sent += 1
        self.goal_pending = True
        self.step_dispatched = True
        self.cancel_requested = False
        self.activity.begin_attempt()
        self.require_controller_evidence = False
        seq = self.goal_seq
        self.goal_handle = None
        self.enter(AWAIT_RESPONSE)
        self.set_wall_deadline(self.goal_response_timeout)
        try:
            future = self.navigator.send_goal_async(goal_msg)
        except Exception as exc:
            self.halt(f'Goal request error: {exc}')
            return
        future.add_done_callback(lambda f: self.goal_response_callback(f, seq))

    def goal_response_callback(self, future, seq):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if seq == self.goal_seq and self.state == AWAIT_RESPONSE:
                self.halt(f'Goal response error: {exc}')
            return

        if seq != self.goal_seq or self.state not in (AWAIT_RESPONSE, HALT_OBSERVE):
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        self.goal_pending = False
        if not goal_handle.accepted:
            self.halt('Goal rejected by Nav2')
            return

        self.get_logger().info('Goal accepted by Nav2.')
        self.goal_handle = goal_handle
        self.goal_active = True
        if self.state == HALT_OBSERVE:
            self.request_cancel()
        else:
            self.enter(NAVIGATING)
            self.set_sim_deadline(self.nav_timeout)
        try:
            result = goal_handle.get_result_async()
        except Exception as exc:
            self.halt(f'Navigation result request failed: {exc}')
            return
        result.add_done_callback(lambda f: self.goal_result_callback(f, seq))

    def tick_await_response(self):
        if self.wall_deadline_passed():
            self.halt(
                f'No goal response from Nav2 within {self.goal_response_timeout:g} s (wall); '
                'goal state unknown')

    def request_cancel(self):
        if self.goal_active and self.goal_handle is not None and not self.cancel_requested:
            self.cancel_requested = True
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f'Cancel request failed: {exc}')

    def tick_navigating(self):
        if self.sim_deadline_passed():
            self.get_logger().error(
                f"Navigation to {self.wp['name']} exceeded {self.nav_timeout:g} s; "
                'requesting cancel.')
            self.enter(CANCELING)
            self.set_wall_deadline(self.cancel_timeout)
            self.request_cancel()

    def tick_canceling(self):
        if self.wall_deadline_passed():
            self.halt(
                f"Navigation to {self.wp['name']} timed out and the cancel was not "
                f'confirmed within {self.cancel_timeout:g} s (wall)')

    def goal_result_callback(self, future, seq):
        if seq != self.goal_seq:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.halt(f'Navigation result error: {exc}')
            return

        if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                          GoalStatus.STATUS_CANCELED):
            self.halt(f'Navigation result has no terminal status: {status}')
            return
        self.goal_active = False
        if self.state not in (NAVIGATING, CANCELING):
            return
        wp = self.wp
        if self.state == CANCELING:
            self.halt(
                f"Navigation to {wp['name']} timed out after {self.nav_timeout:g} s "
                f'(goal ended with status {status} after cancel)')
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.require_controller_evidence = status == GoalStatus.STATUS_ABORTED
            self.get_logger().error(f"Navigation FAILED for {wp['name']} (status: {status})")
            if status == GoalStatus.STATUS_ABORTED and self.nav_attempt <= self.nav_retries:
                self.motion.reset_hold()
                self.enter(RETRY_PAUSE)
                self.set_sim_deadline(self.retry_pause)
                return
            self.halt(
                f"Navigation ended at {wp['name']} "
                f'(status: {status}, attempts: {self.nav_attempt})')
            return

        self.get_logger().info(f"ARRIVED: {wp['name']}")
        self.start_verification()

    def tick_retry_pause(self):
        if self.sim_deadline_passed():
            self.enter_gate(retry=True)

    # ------------------------------------------------------- verification

    def start_verification(self):
        wp = self.wp
        if wp['marker_id'] is None:
            self.get_logger().info('No visual marker required for this waypoint.')
            self.on_verified()
            return
        self.get_logger().info(f"Looking for ArUco marker ID {wp['marker_id']}...")
        self.marker_confirmed = False  # only detections after arrival count
        self.confirmed_marker_id = None
        self.enter(VERIFYING)
        self.set_sim_deadline(self.verify_timeout)

    def tick_verifying(self):
        wp = self.wp
        if self.marker_confirmed:
            self.get_logger().info(
                f"Correct marker detected: expected={wp['marker_id']}, "
                f'detected={self.confirmed_marker_id}')
            self.on_verified()
        elif self.sim_deadline_passed():
            self.get_logger().error(
                f"Marker verification timeout: expected={wp['marker_id']}, "
                f'last_detected={self.last_marker_id}')
            self.get_logger().error(f"VISION VERIFICATION FAILED: {wp['name']}")
            self.failed_tasks += 1
            self.enter(PAUSE)
            self.set_sim_deadline(self.inter_goal_pause)

    def on_verified(self):
        self.get_logger().info(f"VISION VERIFIED: {self.wp['name']}")
        self.get_logger().info(f"TASK: {self.wp['task']}")
        self.enter(TASK_HOLD)
        self.set_sim_deadline(self.task_hold)

    def tick_task_hold(self):
        if self.sim_deadline_passed():
            self.get_logger().info(f"TASK COMPLETE: {self.wp['name']}")
            self.successful_tasks += 1
            self.enter(PAUSE)
            self.set_sim_deadline(self.inter_goal_pause)

    def tick_pause(self):
        if self.sim_deadline_passed():
            self.current_index += 1
            self.step_dispatched = False
            if self.current_index >= len(self.waypoints):
                self.finish_mission()
            else:
                self.enter_gate(retry=False)

    # ------------------------------------------------------------- ending

    def log_summary(self, log_fn, unattempted):
        self.unattempted_tasks = unattempted
        log_fn(f'Total mission steps: {len(self.waypoints)}')
        log_fn(f'Successful tasks: {self.successful_tasks}')
        log_fn(f'Failed tasks: {self.failed_tasks}')
        log_fn(f'Unattempted tasks: {unattempted}')
        log_fn('========================================')

    def finish_mission(self):
        log = self.get_logger()
        log.info('========================================')
        if self.failed_tasks == 0 and self.successful_tasks == len(self.waypoints):
            log.info('WAREHOUSE MISSION COMPLETE: ALL TASKS SUCCEEDED')
            self.log_summary(log.info, 0)
            self.end(EXIT_SUCCESS)
        else:
            log.error(
                f'WAREHOUSE MISSION FINISHED WITH FAILURES: '
                f'{self.failed_tasks} of {len(self.waypoints)} failed')
            self.log_summary(log.info, 0)
            self.end(EXIT_FAILURES)

    def not_started(self, reason):
        log = self.get_logger()
        log.error('========================================')
        log.error(f'MISSION NOT STARTED: {reason}')
        self.log_summary(log.error, len(self.waypoints))
        self.end(EXIT_NOT_STARTED)

    def halt(self, reason):
        if self.state in (HALT_OBSERVE, FINISHED):
            return
        completed = self.successful_tasks + self.failed_tasks
        if self.step_dispatched and completed == self.current_index:
            self.failed_tasks += 1
        self.halt_reason = reason
        log = self.get_logger()
        log.error('========================================')
        log.error(f'MISSION HALTED: {reason}')
        remaining = len(self.waypoints) - self.successful_tasks - self.failed_tasks
        self.log_summary(log.error, remaining)
        log.error('No further mission goals will be sent.')
        if self.goal_active and self.goal_handle is not None:
            log.warn('A navigation goal may still be active; requesting cancel.')
            self.request_cancel()
        self.motion.reset_hold()
        self.enter(HALT_OBSERVE)
        self.set_wall_deadline(self.halt_observe_timeout)

    def tick_halt_observe(self):
        """Report whether the robot is observed stationary and Nav2 idle."""
        wall = self.wall_ns()
        odom_fresh = self.motion.fresh(wall, self.odom_max_age)
        stopped = odom_fresh and self.motion.stationary_for_wall(wall, self.stop_hold)
        active = self.activity.active_actions()
        missing = self.publisher_problems(self.status_publishers())
        idle = not (active or missing or self.goal_active or self.goal_pending)
        if stopped and idle:
            self.stop_verified = True
            self.get_logger().warn(
                'ROBOT STOP VERIFIED (odometry): stationary and no active Nav2 goals.')
            self.end(EXIT_HALTED)
        elif self.wall_deadline_passed():
            reasons = []
            if not odom_fresh:
                reasons.append('no recent odometry')
            elif not stopped:
                reasons.append('stationary odometry hold incomplete')
            if missing:
                reasons.extend(missing)
            if self.goal_pending or self.goal_active:
                reasons.append("navigation goal outcome unresolved")
            if active:
                reasons.append('Nav2 goals still active: ' + ', '.join(sorted(active)))
            self.stop_verified = False
            self.get_logger().error(
                f'ROBOT STOP NOT VERIFIED after {self.halt_observe_timeout:g} s: '
                + '; '.join(reasons))
            self.end(EXIT_HALTED)

    def end(self, code):
        self.enter(FINISHED)
        self.sim_deadline = None
        self.wall_deadline = None
        self.exit_code = code
        self.timer.cancel()

    def destroy_node(self):
        # rclpy's Node.destroy_node() does not destroy action clients.
        self.navigator.destroy()
        super().destroy_node()

    def cancel_active_goal_blocking(self, timeout_sec):
        """On interrupt: run the same bounded cleanup used after a halt."""
        self.halt('Mission interrupted')
        self.set_wall_deadline(timeout_sec)
        while rclpy.ok() and not self.finished:
            rclpy.spin_once(self, timeout_sec=TICK_PERIOD_SEC)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Keep the ROS context alive on SIGINT/SIGTERM so an active goal can be
    # canceled before exiting.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = WarehouseAutonomy()
    exit_code = EXIT_INTERRUPTED

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=TICK_PERIOD_SEC)
        if node.finished:
            exit_code = node.exit_code
    except KeyboardInterrupt:
        try:
            node.cancel_active_goal_blocking(node.cancel_timeout)
        except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
