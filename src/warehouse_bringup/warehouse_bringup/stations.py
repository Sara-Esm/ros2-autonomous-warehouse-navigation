"""
Station approach poses and the map/world frame calibration.

Nav2 executes goals in the SLAM ``map`` frame. The station approach poses
below were measured in Gazebo's ``world`` frame: each is a pose about
0.67-0.97 m in front of its ArUco marker, facing it. The two frames differ
by a rigid transform; the SLAM map's orientation is arbitrary, which is not a
defect in itself. Supplying world-frame coordinates as map-frame goals
produced 0.18-0.50 m stopping errors in the v1.0.1 diagnostic experiment
(docs/diagnostics/REPORT.md, section 2).

Calibration convention (measured, docs/diagnostics/data/run/rigid_alignment.txt,
fit A, from map obstacles against the true world geometry)::

    world = R(theta) * map + t,    theta = +2.05 deg,  t = (-0.142, -0.049) m
    map   = R(-theta) * (world - t)
    yaw_map = yaw_world - theta

This calibration uses simulator ground truth, which is appropriate for the
controlled simulation experiment. On a real robot there is no world-frame
ground truth: station goals would normally be recorded directly in the map
frame, for example by driving the robot to each approach pose and saving its
validated localization estimate. The map-frame values produced here play
that role.
"""

import math

# world = R(theta) * map + t  (geometry-based rigid fit, v1.0.1 diagnostics)
WORLD_FROM_MAP_YAW_DEG = 2.05
WORLD_FROM_MAP_T = (-0.142, -0.049)

# Intended approach poses in the Gazebo world frame: (x [m], y [m], yaw [deg]).
# Unchanged reference data; the ArUco markers stay where the world file puts them.
STATIONS_WORLD = {
    'home_dock': (-3.678, 9.3, 90.0),
    'loading_zone': (1.803, 9.3, 90.0),
    'inventory_scan': (1.801, -9.0, -90.0),
    'dispatch_zone': (-3.630, -9.3, -90.0),
}


def normalize_deg(angle):
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def map_to_world(x, y, yaw_deg):
    """Return the world-frame pose of a map-frame pose (x, y in m, yaw in deg)."""
    th = math.radians(WORLD_FROM_MAP_YAW_DEG)
    tx, ty = WORLD_FROM_MAP_T
    return (math.cos(th) * x - math.sin(th) * y + tx,
            math.sin(th) * x + math.cos(th) * y + ty,
            normalize_deg(yaw_deg + WORLD_FROM_MAP_YAW_DEG))


def world_to_map(x, y, yaw_deg):
    """Return the map-frame pose of a world-frame pose (inverse of map_to_world)."""
    th = math.radians(WORLD_FROM_MAP_YAW_DEG)
    tx, ty = WORLD_FROM_MAP_T
    dx, dy = x - tx, y - ty
    return (math.cos(th) * dx + math.sin(th) * dy,
            -math.sin(th) * dx + math.cos(th) * dy,
            normalize_deg(yaw_deg - WORLD_FROM_MAP_YAW_DEG))


def map_goal(station):
    """Return the map-frame goal for a station, rounded to 1 mm and 0.01 deg."""
    x, y, yaw = world_to_map(*STATIONS_WORLD[station])
    return round(x, 3), round(y, 3), round(yaw, 2)


def _waypoint(name, task, station, marker_id):
    x, y, yaw = map_goal(station)
    return {'name': name, 'task': task, 'station': station,
            'x': x, 'y': y, 'yaw': yaw, 'marker_id': marker_id}


# Mission order. Step 5 reuses the home_dock station, so it is the same pose
# as step 1 by construction.
WAYPOINTS = [
    _waypoint('Home Charging Dock', 'Verify robot home dock and charging station',
              'home_dock', 0),
    _waypoint('Loading Zone', 'Verify loading area', 'loading_zone', 1),
    _waypoint('Inventory Scan Station', 'Verify shelf inventory using wall marker',
              'inventory_scan', 2),
    _waypoint('Dispatch Zone', 'Verify dispatch area', 'dispatch_zone', 3),
    _waypoint('Return to Home Charging Dock',
              'Return robot to charging dock and close mission', 'home_dock', 0),
]
