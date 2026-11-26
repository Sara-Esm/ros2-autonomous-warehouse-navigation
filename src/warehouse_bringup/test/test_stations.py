"""Unit tests for the surveyed map-frame station goals (no ROS graph needed)."""

import math
from pathlib import Path
import re

import pytest

from warehouse_bringup import stations
from warehouse_bringup.stations import normalize_deg, STATIONS_MAP, STATIONS_WORLD, WAYPOINTS
from warehouse_bringup.warehouse_autonomy_node import WAYPOINTS as NODE_WAYPOINTS

WORLD_FILE = (Path(__file__).resolve().parents[2]
              / 'warehouse_gazebo' / 'worlds' / 'small_warehouse.world')


def angle_diff(a, b):
    return abs(normalize_deg(a - b))


def test_waypoints_use_surveyed_map_goals_directly():
    for wp in WAYPOINTS:
        assert (wp['x'], wp['y'], wp['yaw']) == STATIONS_MAP[wp['station']], wp['name']


def test_no_ground_truth_transform_in_operational_module():
    # The Phase 1 simulator-ground-truth calibration (git 69befa2) must not generate goals.
    for name in ('world_to_map', 'map_to_world', 'WORLD_FROM_MAP_YAW_DEG', 'WORLD_FROM_MAP_T'):
        assert not hasattr(stations, name), name


def test_return_home_uses_same_pose_as_home_dock():
    first, last = WAYPOINTS[0], WAYPOINTS[-1]
    assert first['station'] == last['station'] == 'home_dock'
    assert (first['x'], first['y'], first['yaw']) == (last['x'], last['y'], last['yaw'])
    assert first['marker_id'] == last['marker_id'] == 0


def test_mission_order_and_markers():
    assert [wp['marker_id'] for wp in WAYPOINTS] == [0, 1, 2, 3, 0]
    assert [wp['station'] for wp in WAYPOINTS] == [
        'home_dock', 'loading_zone', 'inventory_scan', 'dispatch_zone', 'home_dock']
    assert all(wp.keys() >= {'name', 'task', 'x', 'y', 'yaw', 'marker_id'} for wp in WAYPOINTS)


def test_node_uses_station_module_waypoints():
    assert NODE_WAYPOINTS is WAYPOINTS


def test_goal_headings_face_the_marker_walls():
    for station, (_, y, yaw) in STATIONS_MAP.items():
        expected = 90.0 if y > 0 else -90.0
        assert angle_diff(yaw, expected) < 3.0, station


def test_surveyed_goals_are_near_intended_approach_poses():
    # Loose guard against transcription errors: the map frame is close to the world frame
    # (within about 1 deg and a few cm), so surveyed goals lie near the intended poses.
    for station, (x, y, yaw) in STATIONS_MAP.items():
        wx, wy, wyaw = STATIONS_WORLD[station]
        assert math.hypot(x - wx, y - wy) < 0.25, station
        assert angle_diff(yaw, wyaw) < 3.0, station


def test_normalize_deg_range():
    assert normalize_deg(-179.0 - 2.0) == pytest.approx(179.0)
    for a in (-540.0, -180.0, 180.0, 359.99, 720.0):
        assert -180.0 <= normalize_deg(a) < 180.0


@pytest.mark.skipif(not WORLD_FILE.exists(), reason='world file not found next to this package')
def test_intended_approach_poses_face_their_markers():
    # Markers are not moved; each intended approach pose stands 0.6-1.0 m in front of its marker.
    text = WORLD_FILE.read_text()
    pattern = (r'model://aruco_marker_(\d)</uri>\s*<name>[^<]*</name>'
               r'\s*<pose>([^<]+)</pose>')
    markers = {int(mid): tuple(float(v) for v in pose.split()[:2])
               for mid, pose in re.findall(pattern, text)}
    assert set(markers) == {0, 1, 2, 3}
    for wp in WAYPOINTS:
        sx, sy, syaw = STATIONS_WORLD[wp['station']]
        mx, my = markers[wp['marker_id']]
        assert 0.6 < math.hypot(sx - mx, sy - my) < 1.0, wp['name']
        bearing = math.degrees(math.atan2(my - sy, mx - sx))
        assert angle_diff(bearing, syaw) < 1.0, wp['name']
