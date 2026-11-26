"""
Map classification and unknown-space configuration tests.

The integration test loads the installed warehouse map with the real nav2 map_server and checks
the OccupancyGrid it publishes: the PGM's never-observed value 205 must load as unknown (-1).
The YAML threshold that achieves this is empirical (see warehouse_map.yaml), so the loaded grid,
not the threshold arithmetic, is what is tested.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

from warehouse_bringup.stations import WAYPOINTS

PKG = Path(__file__).resolve().parents[1]
MAP_YAML = PKG / 'maps' / 'warehouse_map.yaml'
MAP_PGM = PKG / 'maps' / 'warehouse_map.pgm'
NAV2_PARAMS = (PKG.parents[0] / 'turtlebot3' / 'turtlebot3_navigation2' / 'param' / 'humble'
               / 'burger_cam.yaml')
EXPECTED = {100: 3772, 0: 51911, -1: 83919}     # occupied, free, unknown cells


def read_pgm(path):
    data = path.read_bytes()
    parts, i = [], 0
    while len(parts) < 4:                         # magic, width, height, maxval
        while data[i:i + 1].isspace():
            i += 1
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    assert parts[0] == b'P5'
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)


def map_meta():
    return yaml.safe_load(MAP_YAML.read_text())


# ------------------------------------------------------------- static checks

def test_pgm_contains_only_trinary_values():
    values, counts = np.unique(read_pgm(MAP_PGM), return_counts=True)
    assert dict(zip(values.tolist(), counts.tolist())) == {0: 3772, 205: 83919, 254: 51911}


def test_map_geometry_unchanged():
    m = map_meta()
    assert m['resolution'] == 0.05
    assert m['origin'] == [-7.56, -10.8, 0]
    assert m['mode'] == 'trinary' and m['negate'] == 0 and m['occupied_thresh'] == 0.65


def test_free_threshold_inside_verified_window():
    # Measured with nav2_map_server 1.1.18 and 1.1.20: value 254 loads free for any
    # free_thresh >= 0.0005, value 205 loads unknown for any free_thresh <= 0.025.
    assert 0.0005 <= map_meta()['free_thresh'] <= 0.025


def test_station_goals_are_on_known_free_cells():
    img = read_pgm(MAP_PGM)
    h = img.shape[0]
    m = map_meta()
    ox, oy, _ = m['origin']
    res = m['resolution']
    occ_r, occ_c = np.nonzero(img == 0)
    for wp in WAYPOINTS:
        mx = math.floor((wp['x'] - ox) / res)          # Nav2 Costmap2D::worldToMap convention
        my = math.floor((wp['y'] - oy) / res)
        row = h - 1 - my
        assert img[row, mx] == 254, wp['name']
        clearance = np.min(np.hypot(occ_r - row, occ_c - mx)) * res
        assert clearance > 0.14, wp['name']             # outside the inscribed radius


@pytest.mark.skipif(not NAV2_PARAMS.exists(), reason='turtlebot3_navigation2 source not found')
def test_unknown_space_policy_configuration():
    p = yaml.safe_load(NAV2_PARAMS.read_text())
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    assert gc['track_unknown_space'] is True
    assert gc['plugins'][0] == 'static_layer'
    # Phase 2a intermediate policy: unknown cells are represented and penalized by NavFn,
    # but still traversable. Phase 2b (re-mapping) is expected to set this to False.
    assert p['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is True


# --------------------------------------------------------- integration test

def _map_server_available():
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix('nav2_map_server')
        return True
    except Exception:
        return False


@pytest.mark.skipif(not _map_server_available(), reason='nav2_map_server is not installed')
def test_loaded_occupancy_grid_matches_expected_classification():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid

    proc = subprocess.Popen(
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
         '-r', '__node:=classification_test_map_server',
         '-p', f'yaml_filename:={MAP_YAML}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
        env=dict(os.environ))
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('map_classification_test', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    try:
        cli = node.create_client(ChangeState, '/classification_test_map_server/change_state')
        assert cli.wait_for_service(timeout_sec=30.0), 'map_server did not start'
        for tid in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            fut = cli.call_async(ChangeState.Request(transition=Transition(id=tid)))
            executor.spin_until_future_complete(fut, timeout_sec=30.0)
            assert fut.result() is not None and fut.result().success, f'transition {tid} failed'
        grids = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, '/map', grids.append, qos)
        deadline = time.monotonic() + 30.0
        while not grids and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
        assert grids, 'no /map received'
        grid = grids[0]
        assert (grid.info.width, grid.info.height) == (318, 439)
        assert grid.info.resolution == pytest.approx(0.05)
        origin = (grid.info.origin.position.x, grid.info.origin.position.y)
        assert origin == pytest.approx((-7.56, -10.8))
        values, counts = np.unique(np.array(grid.data, dtype=np.int16), return_counts=True)
        assert dict(zip(values.tolist(), counts.tolist())) == EXPECTED
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(proc.pid, signal.SIGKILL)
