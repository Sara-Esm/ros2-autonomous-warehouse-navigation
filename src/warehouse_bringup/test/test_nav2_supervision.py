"""Unit tests for the passive supervision helpers (no ROS graph needed)."""

from action_msgs.msg import GoalStatus

from warehouse_bringup.nav2_supervision import ActionActivityTracker, MotionTracker

S = 1_000_000_000


def statuses(*codes):
    return [GoalStatus(status=c) for c in codes]


def test_activity_tracker_reports_only_active_goals():
    t = ActionActivityTracker()
    assert t.idle()
    t.update('follow_path', statuses(GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_SUCCEEDED))
    t.update('spin', statuses(GoalStatus.STATUS_ABORTED))
    assert t.active_actions() == {'follow_path': 1}
    t.update('follow_path', statuses(GoalStatus.STATUS_CANCELING))
    assert not t.idle()
    t.update('follow_path', statuses(GoalStatus.STATUS_CANCELED))
    assert t.idle()


def test_motion_tracker_requires_continuous_stillness():
    m = MotionTracker(linear_tol=0.01, angular_tol=0.02)
    assert not m.fresh(0, 1.0)
    m.update(0.0, 0.0, 0.0, sim_ns=10 * S, wall_ns=100 * S)
    assert m.fresh(100 * S, 1.0)
    assert not m.stationary_for_sim(10 * S + S // 2, 1.0)
    assert m.stationary_for_sim(11 * S, 1.0)
    m.update(0.0, 0.0, 0.05, sim_ns=11 * S, wall_ns=101 * S)  # rotating
    assert not m.stationary_for_sim(12 * S, 1.0)
    m.update(0.005, 0.0, 0.0, sim_ns=12 * S, wall_ns=102 * S)  # below tolerance
    assert m.stationary_for_wall(103 * S, 1.0)
    assert not m.fresh(105 * S, 1.0)
