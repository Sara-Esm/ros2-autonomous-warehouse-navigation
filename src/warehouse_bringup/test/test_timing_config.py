"""Phase 3 timing configuration tests: BT tree, acknowledgement timeout, node clocks."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
BT_XML = PKG / 'behavior_trees' / 'navigate_to_pose_w_replanning_and_recovery_keep_static.xml'
NAV_LAUNCH = PKG / 'launch' / 'warehouse_navigation.launch.py'
NAV2_PARAMS = (PKG.parents[0] / 'turtlebot3' / 'turtlebot3_navigation2' / 'param' / 'humble'
               / 'burger_cam.yaml')
STOCK_BT = Path('/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/'
                'navigate_to_pose_w_replanning_and_recovery.xml')


def nodes(path):
    return [(e.tag, dict(e.attrib)) for e in ET.parse(path).getroot().iter()]


def test_bt_global_clears_keep_static_layer():
    tree = nodes(BT_XML)
    globals_ = [a for t, a in tree if 'global_costmap' in a.get('service_name', '')]
    assert len(globals_) == 2
    assert all(a['service_name'] == 'global_costmap/clear_except_global_costmap' for a in globals_)
    assert not any(t == 'ClearEntireCostmap' and 'global_costmap' in a.get('service_name', '')
                   for t, a in tree)
    local = [a for t, a in tree if t == 'ClearEntireCostmap']
    assert len(local) == 2 and all('local_costmap' in a['service_name'] for a in local)


@pytest.mark.skipif(not STOCK_BT.exists(), reason='Nav2 stock behavior tree not installed')
def test_bt_differs_from_stock_only_in_global_clears():
    ours, stock = nodes(BT_XML), nodes(STOCK_BT)
    assert len(ours) == len(stock)
    diffs = [(a, b) for a, b in zip(ours, stock) if a != b]
    assert len(diffs) == 2
    for (t1, a1), (t2, a2) in diffs:
        assert (t1, t2) == ('ClearCostmapExceptRegion', 'ClearEntireCostmap')
        assert a1['name'] == a2['name'] and 'global_costmap' in a2['service_name']


@pytest.mark.skipif(not NAV2_PARAMS.exists(), reason='turtlebot3_navigation2 source not found')
def test_acknowledgement_timeout_and_clock_sections():
    p = yaml.safe_load(NAV2_PARAMS.read_text())
    bt = p['bt_navigator']['ros__parameters']
    assert bt['default_server_timeout'] == 200
    assert bt['bt_loop_duration'] == 10
    assert 'default_nav_to_pose_bt_xml' in bt          # placeholder rewritten by the launch file
    for node in ('velocity_smoother', 'smoother_server'):
        assert 'use_sim_time' in p[node]['ros__parameters'], node
    # The BT client node deliberately has no section: its acknowledgement timer stays on wall time.
    assert 'bt_navigator_navigate_to_pose_rclcpp_node' not in p


@pytest.mark.skipif(not NAV2_PARAMS.exists(), reason='turtlebot3_navigation2 source not found')
def test_launch_rewrite_produces_expected_parameters():
    nav2_common = pytest.importorskip('nav2_common.launch')
    from launch import LaunchContext
    bringup = nav2_common.RewrittenYaml(source_file=str(NAV2_PARAMS),
                                        param_rewrites={'default_nav_to_pose_bt_xml': str(BT_XML)},
                                        convert_types=True)
    ctx = LaunchContext()
    path = bringup.perform(ctx)
    # emulate nav2_bringup's own rewrite with use_sim_time:=true on top
    sim = nav2_common.RewrittenYaml(source_file=path, param_rewrites={'use_sim_time': 'true'},
                                    convert_types=True).perform(ctx)
    p = yaml.safe_load(Path(sim).read_text())
    assert p['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == str(BT_XML)
    assert p['bt_navigator']['ros__parameters']['default_server_timeout'] == 200
    for node in ('velocity_smoother', 'smoother_server', 'bt_navigator', 'controller_server'):
        assert p[node]['ros__parameters']['use_sim_time'] is True, node
    assert "'default_nav_to_pose_bt_xml': bt_xml" in NAV_LAUNCH.read_text()
