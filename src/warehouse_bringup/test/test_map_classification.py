"""
Map classification and unknown-space configuration tests.

The integration test loads each map YAML with the real nav2 map_server and compares the published
OccupancyGrid cell by cell with the PGM class: 0 -> occupied (100), 254 -> free (0),
205 -> unknown (-1). The YAML threshold that achieves this is empirical (see the YAML comments), so
the loaded grid, not the threshold arithmetic, is what is tested.
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
MAPS = PKG / 'maps'
OPERATIONAL_MAP = MAPS / 'warehouse_map_v2.yaml'      # Phase 2b Cartographer map
ORIGINAL_MAP = MAPS / 'warehouse_map.yaml'            # preserved v1.0.1 map
NAV_LAUNCH = PKG / 'launch' / 'warehouse_navigation.launch.py'
NAV2_PARAMS = (PKG.parents[0] / 'turtlebot3' / 'turtlebot3_navigation2' / 'param' / 'humble'
               / 'burger_cam.yaml')


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


def meta(path):
    return yaml.safe_load(path.read_text())


def expected_grid(path):
    """Return the expected OccupancyGrid data (row 0 = lowest y) from the PGM classes."""
    img = read_pgm(path.parent / meta(path)['image'])
    assert set(np.unique(img).tolist()) <= {0, 205, 254}
    return np.flipud(np.where(img == 0, 100, np.where(img == 254, 0, -1)))


# ------------------------------------------------------------- static checks

@pytest.mark.parametrize('path', [OPERATIONAL_MAP, ORIGINAL_MAP])
def test_pgm_contains_only_trinary_values(path):
    img = read_pgm(path.parent / meta(path)['image'])
    assert set(np.unique(img).tolist()) == {0, 205, 254}


def test_operational_map_geometry():
    m = meta(OPERATIONAL_MAP)
    img = read_pgm(OPERATIONAL_MAP.parent / m['image'])
    assert img.shape == (428, 258)
    assert m['resolution'] == 0.05
    assert m['origin'] == [-7.28, -10.7, 0]
    assert m['mode'] == 'trinary' and m['negate'] == 0 and m['occupied_thresh'] == 0.65


def test_original_map_preserved():
    m = meta(ORIGINAL_MAP)
    assert read_pgm(ORIGINAL_MAP.parent / m['image']).shape == (439, 318)
    assert m['origin'] == [-7.56, -10.8, 0] and m['resolution'] == 0.05


@pytest.mark.parametrize('path', [OPERATIONAL_MAP, ORIGINAL_MAP])
def test_free_threshold_inside_verified_window(path):
    # Measured with nav2_map_server 1.1.18 and 1.1.20: 254 loads free for free_thresh >= 0.0005,
    # 205 loads unknown for free_thresh <= 0.025.
    assert 0.0005 <= meta(path)['free_thresh'] <= 0.025


def test_navigation_launch_uses_operational_map():
    assert "'warehouse_map_v2.yaml'" in NAV_LAUNCH.read_text()


def test_station_goals_are_on_known_free_cells_of_operational_map():
    m = meta(OPERATIONAL_MAP)
    img = read_pgm(OPERATIONAL_MAP.parent / m['image'])
    h = img.shape[0]
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
    # Phase 2b: the re-mapped map connects all mission routes through known space.
    assert p['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False


# --------------------------------------------------------- integration test

def _map_server_available():
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix('nav2_map_server')
        return True
    except Exception:
        return False


@pytest.mark.skipif(not _map_server_available(), reason='nav2_map_server is not installed')
@pytest.mark.parametrize('path', [OPERATIONAL_MAP, ORIGINAL_MAP])
def test_loaded_occupancy_grid_matches_pgm_classes_cell_by_cell(path):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid

    node_name = 'classification_test_map_server'
    proc = subprocess.Popen(
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
         '-r', f'__node:={node_name}', '-p', f'yaml_filename:={path}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
        env=dict(os.environ))
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('map_classification_test', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    try:
        cli = node.create_client(ChangeState, f'/{node_name}/change_state')
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
        m = meta(path)
        expected = expected_grid(path)
        assert (grid.info.height, grid.info.width) == expected.shape
        assert grid.info.resolution == pytest.approx(m['resolution'])
        origin = (grid.info.origin.position.x, grid.info.origin.position.y)
        assert origin == pytest.approx(tuple(m['origin'][:2]))
        loaded = np.array(grid.data, dtype=np.int16).reshape(expected.shape)
        mismatches = int(np.sum(loaded != expected))
        assert mismatches == 0, f'{mismatches} cells differ from the PGM classes'
        assert np.sum(loaded == -1) > 0                  # unknown space is represented
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            os.killpg(proc.pid, signal.SIGKILL)
