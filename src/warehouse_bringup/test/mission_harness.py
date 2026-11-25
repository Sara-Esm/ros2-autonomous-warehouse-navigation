"""
Gazebo-free test harness for the warehouse mission manager.

Provides:
* FakeClock: publishes /clock at a chosen simulation rate; can be paused.
* MockNav2: a NavigateToPose action server with scriptable per-goal behavior,
  plus odometry, the bt_navigator lifecycle GetState service, follow_path
  action status, and the ArUco marker stream.
* MissionHarness: runs the real WarehouseAutonomy node against both in one
  multithreaded executor.

These mocks test mission logic and supervision only; they say nothing about
physical motion safety.
"""

import itertools
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_action_status_default

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int32

from warehouse_bringup.warehouse_autonomy_node import WAYPOINTS, WarehouseAutonomy

MARKER_BY_XY = {(round(wp['x'], 3), round(wp['y'], 3)): wp['marker_id'] for wp in WAYPOINTS}
STEP_XY = [(round(wp['x'], 3), round(wp['y'], 3)) for wp in WAYPOINTS]

# Every harness gets its own DDS domain, so no entity from an earlier test
# (for example a latched action status) can leak into the next one.
_domain_ids = itertools.count(int(os.environ.get('ROS_DOMAIN_ID', '150')))


def next_domain_id():
    return next(_domain_ids) % 230 + 1


# Short timings so a full mission runs in a few seconds.
FAST_PARAMS = {
    'start_delay': 0.4,
    'verify_timeout': 1.0,
    'task_hold': 0.2,
    'inter_goal_pause': 0.2,
    'retry_pause': 0.3,
    'stop_hold': 0.2,
    'nav_timeout': 4.0,
    'startup_timeout': 4.0,
    'gate_timeout': 3.0,
    'goal_response_timeout': 1.5,
    'cancel_timeout': 1.5,
    'clock_stall_timeout': 1.5,
    'halt_observe_timeout': 1.5,
}


class FakeClock:
    """Publish /clock from a background thread at `rate` x real time."""

    def __init__(self, rate=2.0, start_sec=1000.0):
        self.node = Node('fake_clock')
        self.pub = self.node.create_publisher(Clock, '/clock', 10)
        self.rate = rate
        self.sim = start_sec
        self.paused = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if not self.paused:
                self.sim += (now - last) * self.rate
                msg = Clock()
                msg.clock.sec = int(self.sim)
                msg.clock.nanosec = int((self.sim % 1.0) * 1e9)
                try:
                    self.pub.publish(msg)
                except Exception:
                    return
            last = now
            time.sleep(0.005)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


class MockNav2(Node):
    """
    Scriptable stand-in for Nav2, odometry, and the marker detector.

    behaviors[i] is the behavior of the i-th goal received (0-based); goals
    beyond the list succeed. Behaviors:
      'succeed', 'abort', 'reject',
      'no_response' (goal request never answered until release()),
      'late_accept' (goal accepted only after release()),
      'no_result'   (accepted, never finishes unless canceled),
      'slow_accept' (accepted after 0.5 s, then behaves like 'no_result').
    markers: 'correct' | 'wrong' | 'none' | 'before_arrival', or a dict
      {step_index: mode} with 'correct' as the default. The step index is the
      number of navigations completed so far (retries do not advance it).
    """

    def __init__(self, behaviors=None, markers='correct', with_server=True,
                 lifecycle_active=True, nav_duration=0.3):
        super().__init__('mock_nav2')
        self.behaviors = list(behaviors or [])
        self.markers = markers
        self.nav_duration = nav_duration
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        self.goals = []           # (wall time, (x, y)) per goal request
        self.cancel_requests = []  # wall times
        self.events = []          # (wall time, text)
        self.release_event = threading.Event()
        self.current_marker = None
        self.moving = False
        self.lifecycle_id = (LifecycleState.PRIMARY_STATE_ACTIVE if lifecycle_active
                             else LifecycleState.PRIMARY_STATE_INACTIVE)
        self.follow_path_active = False
        self.successes = 0  # mission step = number of navigations completed so far

        self.marker_pub = self.create_publisher(Int32, '/warehouse/marker_id', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.fp_status_pub = self.create_publisher(
            GoalStatusArray, '/follow_path/_action/status', qos_profile_action_status_default)
        self.create_timer(0.05, self._publish_streams, callback_group=self.cb)
        self.create_service(GetState, '/bt_navigator/get_state', self._get_state,
                            callback_group=self.cb)
        self.server = None
        if with_server:
            self.start_server()

    # --- control from tests -------------------------------------------------

    def start_server(self):
        self.server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb)

    def destroy_node(self):
        # rclpy's Node.destroy_node() does not destroy action servers.
        if self.server is not None:
            self.server.destroy()
        super().destroy_node()

    def release(self):
        self.release_event.set()

    def set_lifecycle(self, active):
        self.lifecycle_id = (LifecycleState.PRIMARY_STATE_ACTIVE if active
                             else LifecycleState.PRIMARY_STATE_INACTIVE)
        self.events.append((time.monotonic(), f'lifecycle_active={active}'))

    def set_follow_path_active(self, active):
        self.follow_path_active = active
        self.events.append((time.monotonic(), f'follow_path_active={active}'))

    def set_moving(self, moving):
        self.moving = moving
        self.events.append((time.monotonic(), f'moving={moving}'))

    def goal_xy(self):
        return [xy for _, xy in self.goals]

    def event_time(self, text):
        return next(t for t, e in self.events if e == text)

    # --- internals ----------------------------------------------------------

    def _behavior(self, index):
        return self.behaviors[index] if index < len(self.behaviors) else 'succeed'

    def _marker_mode(self, step):
        if isinstance(self.markers, dict):
            return self.markers.get(step, 'correct')
        return self.markers

    def _publish_streams(self):
        if self.current_marker is not None:
            self.marker_pub.publish(Int32(data=self.current_marker))
        odom = Odometry()
        odom.twist.twist.linear.x = 0.2 if self.moving else 0.0
        self.odom_pub.publish(odom)
        status = GoalStatusArray()
        if self.follow_path_active:
            status.status_list.append(
                GoalStatus(goal_info=GoalInfo(), status=GoalStatus.STATUS_EXECUTING))
        self.fp_status_pub.publish(status)

    def _get_state(self, request, response):
        response.current_state.id = self.lifecycle_id
        response.current_state.label = (
            'active' if self.lifecycle_id == LifecycleState.PRIMARY_STATE_ACTIVE else 'inactive')
        return response

    def _goal_cb(self, goal_request):
        p = goal_request.pose.pose.position
        with self.lock:
            index = len(self.goals)
            self.goals.append((time.monotonic(), (round(p.x, 3), round(p.y, 3))))
        behavior = self._behavior(index)
        if behavior == 'reject':
            return GoalResponse.REJECT
        if behavior == 'slow_accept':
            time.sleep(0.5)
            return GoalResponse.ACCEPT
        if behavior in ('no_response', 'late_accept'):
            self.release_event.wait(timeout=30.0)
            return GoalResponse.ACCEPT if behavior == 'late_accept' else GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.cancel_requests.append(time.monotonic())
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        with self.lock:
            index = len(self.goals) - 1
        behavior = self._behavior(index)
        xy = self.goals[index][1]
        mode = self._marker_mode(self.successes)
        correct = MARKER_BY_XY[xy]

        self.current_marker = correct if mode == 'before_arrival' else None
        self.moving = True
        endless = behavior in ('no_result', 'late_accept', 'slow_accept')
        deadline = time.monotonic() + (30.0 if endless else self.nav_duration)
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.moving = False
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        self.moving = False

        if behavior == 'abort':
            goal_handle.abort()
            return NavigateToPose.Result()
        self.successes += 1
        self.current_marker = {'correct': correct, 'wrong': (correct + 7) % 50,
                               'none': None, 'before_arrival': None}[mode]
        goal_handle.succeed()
        return NavigateToPose.Result()


class MissionHarness:
    """Run WarehouseAutonomy against MockNav2 and FakeClock."""

    def __init__(self, mock_kwargs=None, params=None, clock_rate=2.0, use_sim_time=True):
        self.domain_id = next_domain_id()
        rclpy.init(domain_id=self.domain_id)
        self.clock = FakeClock(rate=clock_rate) if use_sim_time else None
        self.mock = MockNav2(**(mock_kwargs or {}))
        merged = dict(FAST_PARAMS)
        merged.update(params or {})
        overrides = [Parameter(k, value=v) for k, v in merged.items()]
        overrides.append(Parameter('use_sim_time', value=use_sim_time))
        self.mission = WarehouseAutonomy(parameter_overrides=overrides)
        self.executor = MultiThreadedExecutor(num_threads=6)
        for node in (self.mock, self.mission) + ((self.clock.node,) if self.clock else ()):
            self.executor.add_node(node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return predicate()

    def run(self, timeout=30.0):
        assert self.wait_until(lambda: self.mission.finished, timeout), (
            f'mission did not finish within {timeout} s (state={self.mission.state})')
        return self.mission

    def close(self):
        self.mock.release()
        if self.clock:
            self.clock.stop()
        self.executor.shutdown(timeout_sec=2.0)
        for node in (self.mission, self.mock) + ((self.clock.node,) if self.clock else ()):
            node.destroy_node()
        rclpy.shutdown()
