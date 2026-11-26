"""Unit tests for the map/world station calibration (no ROS graph needed)."""

import math
from pathlib import Path
import random
import re

import pytest

from warehouse_bringup import stations
from warehouse_bringup.stations import (
    map_goal, map_to_world, normalize_deg, STATIONS_WORLD, WAYPOINTS, world_to_map,
)
from warehouse_bringup.warehouse_autonomy_node import WAYPOINTS as NODE_WAYPOINTS

# v1.0.1 map-frame goals (world coordinates supplied as map coordinates) and the
# robot's measured world-frame stopping positions for them (diagnostic run,
# docs/diagnostics/REPORT.md section 2).
V101_GOALS = {
    'home_dock': (-3.678, 9.3), 'loading_zone': (1.803, 9.3),
    'inventory_scan': (1.801, -9.0), 'dispatch_zone': (-3.630, -9.3),
}
V101_MEASURED_STOPS = {
    'home_dock': (-4.132, 9.080), 'loading_zone': (1.399, 9.308),
    'inventory_scan': (1.969, -8.948), 'dispatch_zone': (-3.338, -9.420),
}
WORLD_FILE = (Path(__file__).resolve().parents[2]
              / 'warehouse_gazebo' / 'worlds' / 'small_warehouse.world')


def angle_diff(a, b):
    return abs(normalize_deg(a - b))


# ------------------------------------------------------------ direction

def test_forward_transform_reproduces_measured_v101_stops():
    # world = R * map + t: the old map-frame goals must land where the robot
    # physically stopped (within the 0.03-0.11 m residuals of the fit).
    for name, goal in V101_GOALS.items():
        x, y, _ = map_to_world(*goal, 0.0)
        assert math.dist((x, y), V101_MEASURED_STOPS[name]) < 0.12, name


def test_inverse_direction_would_not_reproduce_measured_stops():
    # Guard against a sign/direction mix-up: applying the inverse to the old
    # goals is far from the measured stops for every station.
    for name, goal in V101_GOALS.items():
        x, y, _ = world_to_map(*goal, 0.0)
        assert math.dist((x, y), V101_MEASURED_STOPS[name]) > 0.3, name


# ------------------------------------------------------------- position

def test_world_translation_maps_to_map_origin():
    x, y, yaw = world_to_map(*stations.WORLD_FROM_MAP_T, stations.WORLD_FROM_MAP_YAW_DEG)
    assert (x, y, yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_map_origin_maps_to_world_translation():
    x, y, yaw = map_to_world(0.0, 0.0, 0.0)
    assert (x, y) == pytest.approx(stations.WORLD_FROM_MAP_T, abs=1e-12)
    assert yaw == pytest.approx(stations.WORLD_FROM_MAP_YAW_DEG)


def test_known_rotation_value():
    # A world point 10 m along +y lies at map x = +10*sin(2.05 deg) (after
    # removing t): the map frame is rotated +2.05 deg relative to the world.
    tx, ty = stations.WORLD_FROM_MAP_T
    x, y, _ = world_to_map(tx, ty + 10.0, 0.0)
    th = math.radians(2.05)
    assert (x, y) == pytest.approx((10 * math.sin(th), 10 * math.cos(th)), abs=1e-9)


def test_transform_preserves_distances():
    names = list(STATIONS_WORLD)
    for a in names:
        for b in names:
            dw = math.dist(STATIONS_WORLD[a][:2], STATIONS_WORLD[b][:2])
            dm = math.dist(world_to_map(*STATIONS_WORLD[a])[:2],
                           world_to_map(*STATIONS_WORLD[b])[:2])
            assert dm == pytest.approx(dw, abs=1e-9)


# ---------------------------------------------------------- orientation

def test_yaw_is_offset_by_calibration_rotation():
    for yaw in (-90.0, 0.0, 90.0):
        assert world_to_map(0.0, 0.0, yaw)[2] == pytest.approx(yaw - 2.05)
        assert map_to_world(0.0, 0.0, yaw)[2] == pytest.approx(yaw + 2.05)


def test_yaw_wraps_into_half_open_range():
    assert world_to_map(0.0, 0.0, -179.0)[2] == pytest.approx(178.95)
    assert map_to_world(0.0, 0.0, 179.0)[2] == pytest.approx(-178.95)
    for a in (-540.0, -180.0, 180.0, 359.99, 720.0):
        assert -180.0 <= normalize_deg(a) < 180.0


# ----------------------------------------------------------- round trip

def test_round_trip_both_directions():
    rng = random.Random(7)
    for _ in range(500):
        p = (rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(-360, 360))
        w = map_to_world(*world_to_map(*p))
        assert w[:2] == pytest.approx(p[:2], abs=1e-9)
        assert angle_diff(w[2], p[2]) < 1e-9
        m = world_to_map(*map_to_world(*p))
        assert m[:2] == pytest.approx(p[:2], abs=1e-9)
        assert angle_diff(m[2], p[2]) < 1e-9


# ------------------------------------------------- station goal consistency

def test_waypoints_are_map_frame_versions_of_world_stations():
    for wp in WAYPOINTS:
        x, y, yaw = map_to_world(wp['x'], wp['y'], wp['yaw'])
        wx, wy, wyaw = STATIONS_WORLD[wp['station']]
        # rounding to 1 mm / 0.01 deg is the only allowed difference
        assert math.dist((x, y), (wx, wy)) < 1e-3, wp['name']
        assert angle_diff(yaw, wyaw) < 0.011, wp['name']


def test_waypoints_differ_from_uncorrected_goals():
    for wp in WAYPOINTS:
        old = V101_GOALS[wp['station']]
        assert math.dist((wp['x'], wp['y']), old) > 0.1, wp['name']


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


def test_map_goal_rounding():
    for name in STATIONS_WORLD:
        x, y, yaw = map_goal(name)
        assert round(x, 3) == x and round(y, 3) == y and round(yaw, 2) == yaw


@pytest.mark.skipif(not WORLD_FILE.exists(), reason='world file not found next to this package')
def test_world_approach_poses_face_their_markers():
    # Markers are not moved; each world-frame approach pose stands 0.6-1.0 m in
    # front of its marker and faces it.
    text = WORLD_FILE.read_text()
    markers = {}
    pattern = (r'model://aruco_marker_(\d)</uri>\s*<name>[^<]*</name>'
               r'\s*<pose>([^<]+)</pose>')
    for mid, pose in re.findall(pattern, text):
        x, y = (float(v) for v in pose.split()[:2])
        markers[int(mid)] = (x, y)
    assert set(markers) == {0, 1, 2, 3}
    for wp in WAYPOINTS:
        sx, sy, syaw = STATIONS_WORLD[wp['station']]
        mx, my = markers[wp['marker_id']]
        assert 0.6 < math.dist((sx, sy), (mx, my)) < 1.0, wp['name']
        bearing = math.degrees(math.atan2(my - sy, mx - sx))
        assert angle_diff(bearing, syaw) < 1.0, wp['name']
