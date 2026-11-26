"""
Station goals in the robot's map frame (warehouse_map_v2).

The goals below were SURVEYED, not computed. In a localization session on warehouse_map_v2
(Phase 2b), the robot was positioned at each intended physical approach pose (about 0.67-0.97 m
in front of its ArUco marker, facing it), and its validated AMCL/TF estimate of
map -> base_footprint was recorded (mean of three stable samples after small yaw dithers).
A real deployment would use the same procedure, with an operator positioning the robot.
See docs/validation/phase2b_slam_remap.

History: Phase 1 (commit 69befa2) derived the goals from Gazebo world coordinates with a
calibration measured from simulator ground truth. That transform is no longer used for navigation.
"""

# Surveyed map-frame goals: (x [m], y [m], yaw [deg]).
# Survey stability (max sample deviation) / validation against simulation truth (evaluation only):
#   home_dock       1.0 cm, 1.5 deg / 1.4 cm from the intended pose
#   loading_zone    1.9 cm, 0.9 deg / 1.8 cm
#   inventory_scan  2.4 cm, 0.5 deg / 7.7 cm
#   dispatch_zone   2.2 cm, 0.3 deg / 3.5 cm
STATIONS_MAP = {
    'home_dock': (-3.543, 9.296, 90.59),
    'loading_zone': (1.933, 9.220, 88.93),
    'inventory_scan': (1.693, -8.985, -90.30),
    'dispatch_zone': (-3.704, -9.282, -91.24),
}

# Intended physical approach poses in Gazebo's world frame. EVALUATION REFERENCE ONLY: used to
# measure positioning accuracy against simulator ground truth, never to generate navigation goals.
STATIONS_WORLD = {
    'home_dock': (-3.678, 9.3, 90.0),
    'loading_zone': (1.803, 9.3, 90.0),
    'inventory_scan': (1.801, -9.0, -90.0),
    'dispatch_zone': (-3.630, -9.3, -90.0),
}


def normalize_deg(angle):
    """Wrap an angle in degrees to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def _waypoint(name, task, station, marker_id):
    x, y, yaw = STATIONS_MAP[station]
    return {'name': name, 'task': task, 'station': station,
            'x': x, 'y': y, 'yaw': yaw, 'marker_id': marker_id}


# Mission order. Step 5 reuses the home_dock station, so it is the same pose as step 1.
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
