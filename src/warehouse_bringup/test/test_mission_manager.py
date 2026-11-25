"""
Mission-manager behavior tests against a mock Nav2 (no Gazebo).

Each test runs the real WarehouseAutonomy node on an isolated ROS domain
(see conftest.py) with a fake /clock. They verify mission logic and
supervision, not physical robot behavior.
"""

import os
import signal
import subprocess
import sys
import time

import pytest

from mission_harness import MissionHarness, STEP_XY
from warehouse_bringup import warehouse_autonomy_node as wan


@pytest.fixture
def harness_factory():
    created = []

    def make(**kwargs):
        h = MissionHarness(**kwargs)
        created.append(h)
        return h

    yield make
    for h in created:
        h.close()


def assert_counts(m, ok, failed, unattempted):
    assert (m.successful_tasks, m.failed_tasks, m.unattempted_tasks) == (ok, failed, unattempted)


# --------------------------------------------------------------- mission flow

def test_all_goals_succeed_five_step_mission(harness_factory):
    h = harness_factory()
    m = h.run()
    assert m.exit_code == wan.EXIT_SUCCESS
    assert_counts(m, 5, 0, 0)
    assert h.mock.goal_xy() == STEP_XY  # exactly five goals, in mission order
    assert m.halt_reason is None


def test_goal_rejected_halts_without_further_goals(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'reject']})
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert_counts(m, 1, 1, 3)
    assert 'rejected' in m.halt_reason
    time.sleep(0.5)
    assert h.mock.goal_xy() == STEP_XY[:2]


def test_navigation_abort_without_retries_halts(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'abort']},
                        params={'nav_retries': 0})
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert_counts(m, 1, 1, 3)
    assert h.mock.goal_xy() == STEP_XY[:2]


def test_single_failure_succeeds_on_retry(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'abort']})
    m = h.run()
    assert m.exit_code == wan.EXIT_SUCCESS
    assert_counts(m, 5, 0, 0)
    goals = h.mock.goal_xy()
    assert goals == [STEP_XY[0], STEP_XY[1], STEP_XY[1]] + STEP_XY[2:]


def test_failure_after_retry_limit_halts_with_no_extra_waypoint(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'abort', 'abort']})
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert_counts(m, 1, 1, 3)
    assert 'attempts: 2' in m.halt_reason
    time.sleep(0.6)  # nothing more may be sent after the terminal failure
    assert h.mock.goal_xy() == [STEP_XY[0], STEP_XY[1], STEP_XY[1]]


# ------------------------------------------------------------- verification

def test_wrong_marker_fails_task_and_mission_continues(harness_factory):
    h = harness_factory(mock_kwargs={'markers': {2: 'wrong'}})
    m = h.run()
    assert m.exit_code == wan.EXIT_FAILURES
    assert_counts(m, 4, 1, 0)
    assert h.mock.goal_xy() == STEP_XY
    assert m.last_marker_id is not None


def test_no_marker_before_deadline_fails_task(harness_factory):
    h = harness_factory(mock_kwargs={'markers': {0: 'none'}})
    m = h.run()
    assert m.exit_code == wan.EXIT_FAILURES
    assert_counts(m, 4, 1, 0)


def test_marker_seen_only_before_arrival_does_not_count(harness_factory):
    h = harness_factory(mock_kwargs={'markers': {1: 'before_arrival'}})
    m = h.run()
    assert m.exit_code == wan.EXIT_FAILURES
    assert_counts(m, 4, 1, 0)


@pytest.mark.parametrize('rate', [0.5, 2.0])
def test_verification_window_uses_simulation_time(harness_factory, rate):
    """A 1.0 s (sim) window must last 1/rate seconds of wall time."""
    h = harness_factory(mock_kwargs={'markers': {0: 'none'}}, clock_rate=rate)
    m = h.mission
    assert h.wait_until(lambda: m.state == wan.VERIFYING, 15.0)
    start = time.monotonic()
    assert h.wait_until(lambda: m.failed_tasks == 1, 10.0)
    elapsed = time.monotonic() - start
    expected = 1.0 / rate
    assert expected * 0.8 <= elapsed <= expected * 1.4 + 0.2, elapsed


# ---------------------------------------------- availability and watchdogs

def test_nav2_unavailable_at_startup_exits_not_started(harness_factory):
    h = harness_factory(mock_kwargs={'with_server': False})
    m = h.run(timeout=15.0)
    assert m.exit_code == wan.EXIT_NOT_STARTED
    assert_counts(m, 0, 0, 5)
    assert h.mock.goals == []


def test_nav2_temporarily_unavailable_then_mission_runs(harness_factory):
    h = harness_factory(mock_kwargs={'with_server': False})
    time.sleep(1.5)
    h.mock.start_server()
    m = h.run()
    assert m.exit_code == wan.EXIT_SUCCESS


def test_bt_navigator_inactive_at_startup_waits_for_active(harness_factory):
    h = harness_factory(mock_kwargs={'lifecycle_active': False})
    time.sleep(1.5)
    assert h.mock.goals == []
    h.mock.set_lifecycle(True)
    m = h.run()
    assert m.exit_code == wan.EXIT_SUCCESS


@pytest.mark.parametrize('behavior,expect_cancel', [('slow_accept', True), ('no_response', False)])
def test_process_goal_response_timeout(behavior, expect_cancel):
    h = _MockOnly([behavior])
    try:
        code, out = _run_process(h, extra_args=('-p', 'goal_response_timeout:=0.1'))
        assert code == wan.EXIT_HALTED, out
        assert 'No goal response' in out
        assert len(h.mock.goals) == 1
        assert len(h.mock.cancel_requests) == int(expect_cancel)
        if expect_cancel:
            assert 'ROBOT STOP VERIFIED' in out
        else:
            assert 'ROBOT STOP NOT VERIFIED' in out
            assert 'navigation goal outcome unresolved' in out
    finally:
        h.close()


def test_external_cancellation_halts_without_retry(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['no_result']})
    assert h.wait_until(lambda: h.mission.state == wan.NAVIGATING, 10.0)
    h.mission.goal_handle.cancel_goal_async()  # unsolicited cancellation at the server
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert_counts(m, 0, 1, 4)
    assert len(h.mock.goals) == 1


def test_pause_halt_preserves_completed_task(harness_factory):
    h = harness_factory(params={'inter_goal_pause': 30.0})
    assert h.wait_until(lambda: h.mission.state == wan.PAUSE, 15.0)
    h.clock.paused = True
    m = h.run()
    assert_counts(m, 1, 0, 4)


def test_retry_without_controller_observation_halts(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['abort'], 'report_controller': False})
    m = h.run()
    assert len(h.mock.goals) == 1
    assert m.exit_code == wan.EXIT_HALTED
    assert 'controller activity' in m.halt_reason
    assert m.stop_verified is False


def test_cancel_rejection_leaves_stop_unverified(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['no_result'], 'accept_cancel': False},
                        params={'nav_timeout': 1.0})
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert m.stop_verified is False
    assert len(h.mock.cancel_requests) == len(h.mock.goals) == 1


def test_result_never_arrives_cancels_goal_and_halts(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'no_result']},
                        params={'nav_timeout': 2.0})
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert 'timed out' in m.halt_reason
    assert len(h.mock.cancel_requests) == 1
    assert_counts(m, 1, 1, 3)
    assert len(h.mock.goals) == 2  # no retry after a timeout


def test_stalled_simulation_clock_is_detected(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['no_result']},
                        params={'nav_timeout': 100.0})
    m = h.mission
    assert h.wait_until(lambda: m.state == wan.NAVIGATING, 10.0)
    h.clock.paused = True
    start = time.monotonic()
    m = h.run(timeout=10.0)
    assert m.exit_code == wan.EXIT_HALTED
    assert 'clock stalled' in m.halt_reason
    assert time.monotonic() - start < 4.0  # wall watchdog, not the sim timeout
    assert len(h.mock.cancel_requests) == 1  # active goal was canceled


# -------------------------------------------- supervision before retrying

def test_retry_waits_for_lower_level_nav2_activity_to_end(harness_factory):
    """Trial 08: follow_path kept executing after NavigateToPose aborted."""
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'abort']})
    m = h.mission
    assert h.wait_until(lambda: len(h.mock.goals) == 2, 10.0)
    h.mock.set_follow_path_active(True)
    assert h.wait_until(lambda: m.state == wan.GATE and m.nav_attempt == 1, 5.0)
    time.sleep(1.0)
    assert len(h.mock.goals) == 2  # gated
    h.mock.set_follow_path_active(False)
    m = h.run()
    assert m.exit_code == wan.EXIT_SUCCESS
    retry_time = h.mock.goals[2][0]
    assert retry_time > h.mock.event_time('follow_path_active=False')


def test_retry_waits_until_robot_is_stationary(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'abort']})
    m = h.mission
    assert h.wait_until(lambda: m.state == wan.RETRY_PAUSE, 10.0)
    h.mock.set_moving(True)
    time.sleep(1.0)
    assert len(h.mock.goals) == 2
    h.mock.set_moving(False)
    m = h.run()
    assert m.exit_code == wan.EXIT_SUCCESS
    assert h.mock.goals[2][0] > h.mock.event_time('moving=False')


def test_navigation_recovers_after_lifecycle_failure(harness_factory):
    """Fault injection 01: controller deactivated, Nav2 later reactivated."""
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'abort']})
    m = h.mission
    assert h.wait_until(lambda: len(h.mock.goals) == 2, 10.0)
    h.mock.set_lifecycle(False)
    assert h.wait_until(lambda: m.state == wan.GATE, 5.0)
    time.sleep(1.0)
    assert len(h.mock.goals) == 2  # no goal while bt_navigator is inactive
    h.mock.set_lifecycle(True)
    m = h.run()
    assert m.exit_code == wan.EXIT_SUCCESS
    assert h.mock.goals[2][0] > h.mock.event_time('lifecycle_active=True')


def test_lifecycle_never_recovers_halts_at_readiness_gate(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['succeed', 'abort']})
    assert h.wait_until(lambda: len(h.mock.goals) == 2, 10.0)
    h.mock.set_lifecycle(False)
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert 'Readiness check failed' in m.halt_reason
    assert len(h.mock.goals) == 2


# -------------------------------------------------- halt stop reporting

def test_halt_reports_verified_stop_when_stationary(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['abort']}, params={'nav_retries': 0})
    m = h.run()
    assert m.exit_code == wan.EXIT_HALTED
    assert m.stop_verified is True


def test_halt_reports_unverified_stop_when_robot_keeps_moving(harness_factory):
    h = harness_factory(mock_kwargs={'behaviors': ['abort']}, params={'nav_retries': 0})
    m = h.mission
    assert h.wait_until(lambda: m.state == wan.NAVIGATING, 10.0)
    h.mock.set_moving(True)
    # The mock clears `moving` when it aborts; keep the robot moving afterwards.
    assert h.wait_until(lambda: m.state == wan.HALT_OBSERVE, 10.0)
    h.mock.set_moving(True)
    m = h.run()
    assert m.stop_verified is False


# ---------------------------------------------------- process exit codes

def _installed_executable():
    """Return the console script that `ros2 run warehouse_bringup warehouse_autonomy` starts."""
    from ament_index_python.packages import get_package_prefix
    path = os.path.join(get_package_prefix('warehouse_bringup'),
                        'lib', 'warehouse_bringup', 'warehouse_autonomy')
    assert os.access(path, os.X_OK), f'installed mission executable not found: {path}'
    return path


def _run_process(h, extra_args=(), interrupt_when=None, timeout=40.0,
                 command=None, interrupt_signal=signal.SIGINT):
    if command is None:
        command = [sys.executable, '-m', 'warehouse_bringup.warehouse_autonomy_node']
    args = list(command) + ['--ros-args', '-p', 'use_sim_time:=true']
    for k, v in [('start_delay', 0.4), ('verify_timeout', 1.0), ('task_hold', 0.2),
                 ('inter_goal_pause', 0.2), ('retry_pause', 0.3), ('stop_hold', 0.2),
                 ('startup_timeout', 8.0), ('halt_observe_timeout', 1.5)]:
        args += ['-p', f'{k}:={v}']
    args += list(extra_args)
    env = dict(os.environ, ROS_DOMAIN_ID=str(h.domain_id))
    proc = subprocess.Popen(args, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    try:
        if interrupt_when is not None:
            assert h.wait_until(interrupt_when, 20.0)
            proc.send_signal(interrupt_signal)
        out, _ = proc.communicate(timeout=timeout)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
    return proc.returncode, out


@pytest.mark.parametrize('behaviors,expected', [
    ([], wan.EXIT_SUCCESS),
    (['succeed', 'abort', 'abort'], wan.EXIT_HALTED),
])
def test_process_exit_codes(behaviors, expected):
    h = _MockOnly(behaviors)
    try:
        code, out = _run_process(h)
    finally:
        h.close()
    assert code == expected, out


def test_process_finished_with_failures_exit_code():
    h = _MockOnly([], markers={3: 'none'})
    try:
        code, out = _run_process(h)
    finally:
        h.close()
    assert code == wan.EXIT_FAILURES, out


@pytest.mark.parametrize('behavior,delay', [
    ('no_result', 1.0),    # interrupt while the goal is executing
    ('slow_accept', 0.0),  # interrupt while goal acceptance is still pending
])
def test_sigint_cancels_active_goal_and_exits_130(behavior, delay):
    h = _MockOnly([behavior])

    def goal_received_then_delay():
        if not h.mock.goals:
            return False
        return time.monotonic() - h.mock.goals[0][0] >= delay

    try:
        code, out = _run_process(h, interrupt_when=goal_received_then_delay)
        cancel_seen = h.wait_until(lambda: len(h.mock.cancel_requests) == 1, 3.0)
    finally:
        h.close()
    assert code == wan.EXIT_INTERRUPTED, out
    assert cancel_seen, out
    assert 'ROBOT STOP VERIFIED' in out


def test_sigterm_to_installed_executable_cancels_goal_and_exits_130():
    # SIGTERM is sent directly to the installed mission process. `ros2 run` is
    # deliberately not used: Humble's ros2 run does not forward signals sent to
    # its own PID to the mission child (see README, "Stopping the mission").
    # Signal once the goal is executing, as in the SIGINT test. A signal that lands at the
    # exact moment of goal acceptance can leave cleanup unverified (README, same section).
    h = _MockOnly(['no_result'])

    def goal_executing_for_1s():
        return bool(h.mock.goals) and time.monotonic() - h.mock.goals[0][0] >= 1.0

    try:
        code, out = _run_process(h, interrupt_when=goal_executing_for_1s,
                                 command=[_installed_executable()],
                                 interrupt_signal=signal.SIGTERM)
        cancel_seen = h.wait_until(lambda: len(h.mock.cancel_requests) == 1, 3.0)
    finally:
        h.close()
    assert code == wan.EXIT_INTERRUPTED, out
    assert cancel_seen, out
    assert 'MISSION HALTED: Mission interrupted' in out
    assert 'ROBOT STOP VERIFIED' in out
    assert len(h.mock.goals) == 1


class _MockOnly(MissionHarness):
    """Mock Nav2 + clock in this process; the mission runs as a subprocess."""

    def __init__(self, behaviors, markers='correct'):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from mission_harness import FakeClock, MockNav2, next_domain_id
        import threading
        self.domain_id = next_domain_id()
        rclpy.init(domain_id=self.domain_id)
        self.clock = FakeClock(rate=2.0)
        self.mock = MockNav2(behaviors=behaviors, markers=markers)
        self.mission = None
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.mock)
        self.executor.add_node(self.clock.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def close(self):
        self.mock.release()
        self.clock.stop()
        self.executor.shutdown()
        self._thread.join(timeout=5.0)
        self.mock.destroy_node()
        self.clock.node.destroy_node()
        import rclpy
        rclpy.shutdown()
