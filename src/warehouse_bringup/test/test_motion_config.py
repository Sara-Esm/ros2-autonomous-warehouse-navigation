"""Phase 5 motion-control configuration: the two tested changes, and what must stay unchanged."""

from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
NAV2_PARAMS = (PKG.parents[0] / 'turtlebot3' / 'turtlebot3_navigation2' / 'param' / 'humble'
               / 'burger_cam.yaml')


def params():
    return yaml.safe_load(NAV2_PARAMS.read_text())


def test_phase5_changes():
    c = params()['controller_server']['ros__parameters']
    assert c['general_goal_checker']['yaw_goal_tolerance'] == 0.07
    assert c['FollowPath']['PathAlign.forward_point_distance'] == 0.325
    assert c['FollowPath']['GoalAlign.forward_point_distance'] == 0.325


def test_unchanged_controller_limits_and_tolerances():
    c = params()['controller_server']['ros__parameters']
    fp = c['FollowPath']
    assert c['controller_frequency'] == 10.0
    assert c['general_goal_checker']['xy_goal_tolerance'] == 0.12
    assert c['general_goal_checker']['plugin'] == 'nav2_controller::SimpleGoalChecker'
    limits = (fp['max_vel_x'], fp['max_vel_theta'], fp['acc_lim_x'], fp['acc_lim_theta'])
    assert limits == (0.22, 1.0, 2.5, 3.2)
    assert fp['critics'] == ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                             'PathAlign', 'PathDist', 'GoalDist']
    assert fp['plugin'] == 'dwb_core::DWBLocalPlanner'


def test_unknown_space_policy_unchanged():
    p = params()
    assert p['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
    assert p['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] is True
    local = p['local_costmap']['local_costmap']['ros__parameters']
    # Nav2 default false; the Phase 5 replay showed strict local tracking blocks DWB
    assert 'track_unknown_space' not in local
    assert 'inf_is_valid' not in local['obstacle_layer']['scan']
