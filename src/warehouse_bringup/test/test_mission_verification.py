"""
Phase 6 mission verification: physical settling, marker-evidence freshness and final check.

Runs the real WarehouseAutonomy node against the mock Nav2 (no Gazebo). The mock
publishes stamped observations on /warehouse/marker_detections and, alongside,
the legacy unstamped Int32 marker topic, which the mission must not trust.
"""

import signal
import time

import pytest

from mission_harness import MissionHarness
from test_mission_manager import _MockOnly as _MockOnlyBase, _run_process
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


def verified(m):
    return [r for r in m.station_records if r['outcome'] == 'verified']


# ------------------------------------------------------- settling and evidence

def test_correct_fresh_observation_after_settling_succeeds(harness_factory):
    m = harness_factory().run()
    assert m.exit_code == wan.EXIT_SUCCESS
    assert len(verified(m)) == 5
    for r in m.station_records:
        # The settle hold (0.2 s here) is counted only after goal success, and the evidence was
        # captured after settling.
        assert r['settled_ns'] - r['arrival_ns'] >= 0.2e9
        assert r['evidence_stamp_ns'] >= r['window_start_ns']


def test_navigation_succeeds_while_robot_still_moving_waits_for_settling(harness_factory):
    # The mock keeps moving 1.0 s sim after every goal success.
    h = harness_factory(mock_kwargs={'post_arrival_motion': 1.0})
    m = h.run(timeout=60.0)
    assert m.exit_code == wan.EXIT_SUCCESS
    stops = h.mock.post_arrival_stops_ns
    assert len(stops) == len(m.station_records) == 5
    for r, stopped_ns in zip(m.station_records, stops):
        # Settling is credited only after the robot actually stopped, plus the 0.2 s hold.
        assert r['settled_ns'] >= stopped_ns + 0.2e9
        assert r['evidence_stamp_ns'] >= r['window_start_ns']


def test_robot_never_stationary_fails_task_and_never_verifies(harness_factory):
    h = harness_factory(mock_kwargs={'post_arrival_motion': None})
    m = h.run(timeout=30.0)
    assert m.successful_tasks == 0
    assert m.failed_tasks >= 1
    assert 'STATION SETTLE FAILED' in m.station_records[0]['outcome']
    assert m.exit_code in (wan.EXIT_HALTED, wan.EXIT_FAILURES)
    assert not verified(m)


@pytest.mark.parametrize('mode,reason', [
    ('stale_queued', 'captured before verification window'),  # image taken before arrival
    ('delayed', 'too old'),        # captured inside the window, delivered beyond marker_max_age
    ('wrong', 'wrong id'),
    ('legacy_only', 'no marker'),        # correct ID only on the unstamped legacy topic
])
def test_untrustworthy_marker_evidence_never_verifies(harness_factory, mode, reason):
    # A 4 s window lets delayed observations of in-window images arrive before it closes.
    h = harness_factory(mock_kwargs={'markers': {0: mode}}, params={'verify_timeout': 4.0})
    m = h.run(timeout=40.0)
    first = m.station_records[0]
    assert first['outcome'].startswith('VISION VERIFICATION FAILED')
    assert first['evidence_stamp_ns'] is None
    assert first['rejections'].get(reason, 0) > 0, first['rejections']
    assert m.failed_tasks == 1 and m.successful_tasks == 4
    assert m.exit_code == wan.EXIT_FAILURES


def test_stale_odometry_is_not_proof_of_stationarity(harness_factory):
    # Zero-velocity odometry, but stamped 5 s in the past: rejected, so the robot is
    # never considered settled (or ready), and nothing is verified.
    h = harness_factory(mock_kwargs={'odom_stamp_offset': -5.0})
    m = h.run(timeout=30.0)
    assert m.successful_tasks == 0 and not verified(m)
    assert m.exit_code == wan.EXIT_HALTED
    assert 'no recent odometry' in (m.halt_reason or '')


def test_odometry_lost_after_arrival_fails_settling(harness_factory):
    # Odometry ends deterministically when the mock receives the first goal, so no
    # zero-velocity message from after the goal can be credited as settling.
    h = harness_factory()
    h.mock.stop_odom_on_first_goal = True
    m = h.run(timeout=30.0)
    assert m.station_records[0]['outcome'].startswith('STATION SETTLE FAILED')
    assert 'odometry' in m.station_records[0]['outcome']
    assert not verified(m)


def test_clock_stall_during_settling_halts(harness_factory):
    h = harness_factory(mock_kwargs={'post_arrival_motion': None})
    assert h.wait_until(lambda: h.mission.state == wan.SETTLING, 15.0)
    h.clock.paused = True
    m = h.run(timeout=20.0)
    assert m.exit_code == wan.EXIT_HALTED
    assert 'Simulation clock stalled' in m.halt_reason
    assert not verified(m)


def test_final_check_requires_nav2_idle_before_reporting_success(harness_factory):
    h = harness_factory()
    last = len(wan.WAYPOINTS) - 1
    assert h.wait_until(lambda: h.mission.current_index == last and
                        h.mission.state == wan.TASK_HOLD, 40.0)
    h.mock.set_action_active('spin', True)
    m = h.run(timeout=20.0)
    assert m.successful_tasks == 5
    assert m.completion_confirmed is False
    assert m.exit_code == wan.EXIT_FAILURES


def test_final_check_requires_robot_stationary(harness_factory):
    h = harness_factory()
    last = len(wan.WAYPOINTS) - 1
    assert h.wait_until(lambda: h.mission.current_index == last and
                        h.mission.state == wan.TASK_HOLD, 40.0)
    h.mock.set_moving(True)
    m = h.run(timeout=20.0)
    assert m.completion_confirmed is False
    assert m.exit_code == wan.EXIT_FAILURES


# --------------------------------------------- shutdown in each mission phase

class _MockOnly(_MockOnlyBase):
    def __init__(self, behaviors, markers='correct', **mock_kwargs):
        super().__init__(behaviors, markers)
        for k, v in mock_kwargs.items():
            setattr(self.mock, k, v)


@pytest.mark.parametrize('phase', ['idle', 'settling', 'verifying'])
def test_sigint_in_each_mission_phase_exits_130_without_new_goals(phase):
    kwargs, extra, when = {}, [], None
    if phase == 'idle':
        extra = ['-p', 'start_delay:=60.0']
    if phase == 'settling':
        kwargs = {'post_arrival_motion': None}   # never settles: interrupt while SETTLING
        extra = ['-p', 'settle_timeout:=60.0']
    if phase == 'verifying':
        kwargs = {'markers': {0: 'none'}}         # settled, waiting for evidence
        extra = ['-p', 'verify_timeout:=60.0', '-p', 'settle_hold:=0.2']
    h = _MockOnly([], **kwargs)
    if phase == 'idle':
        # Up and idle (in the start delay): it has polled bt_navigator's state several times.
        when = (lambda: h.mock.get_state_calls >= 3)
    else:
        when = (lambda: h.mock.successes >= 1 and time.monotonic() - h.mock.goals[0][0] > 2.0)
    try:
        code, out = _run_process(h, extra_args=extra, interrupt_when=when,
                                 interrupt_signal=signal.SIGINT)
        goals = len(h.mock.goals)
    finally:
        h.close()
    assert code == wan.EXIT_INTERRUPTED, out
    assert 'MISSION HALTED: Mission interrupted' in out, out
    assert goals == (0 if phase == 'idle' else 1), out
    if phase == 'settling':
        assert 'ROBOT STOP NOT VERIFIED' in out, out   # the mock robot is still moving
    else:
        assert 'ROBOT STOP VERIFIED' in out, out


@pytest.mark.parametrize('delay', [0.15, 0.45, 0.8, 1.3, 2.1])
def test_sigint_at_arbitrary_points_always_runs_cleanup(delay):
    """
    Interrupt at several points; every one must run the bounded cleanup and exit 130.

    A signal can land anywhere in the main thread. It used to be able to abort a C-level
    message take (exit 1, no cleanup); now it only sets a flag that the loop acts on.
    """
    h = _MockOnly([])

    def first_goal_plus_delay():
        return bool(h.mock.goals) and time.monotonic() - h.mock.goals[0][0] >= delay

    try:
        code, out = _run_process(h, interrupt_when=first_goal_plus_delay,
                                 interrupt_signal=signal.SIGINT)
    finally:
        h.close()
    assert code == wan.EXIT_INTERRUPTED, out
    assert 'SIGINT received: stopping the mission.' in out, out
    assert 'MISSION HALTED: Mission interrupted' in out, out
    assert 'ROBOT STOP VERIFIED' in out or 'ROBOT STOP NOT VERIFIED' in out, out
    assert 'Traceback' not in out, out
