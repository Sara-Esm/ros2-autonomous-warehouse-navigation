"""Phase 4b headless launch options: graphical defaults kept, each window optional."""

import ast
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / 'launch'
SYSTEM = LAUNCH / 'warehouse_robot_system_camera.launch.py'
SIM = LAUNCH / 'warehouse_simulation_camera.launch.py'
NAV = LAUNCH / 'warehouse_navigation.launch.py'
VENDORED_NAV2 = (PKG.parents[0] / 'turtlebot3' / 'turtlebot3_navigation2' / 'launch'
                 / 'navigation2.launch.py')


def tree(path):
    return ast.parse(path.read_text())


def declared_defaults(path):
    out = {}
    for n in ast.walk(tree(path)):
        if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'DeclareLaunchArgument':
            kw = {k.arg: k.value for k in n.keywords}
            out[n.args[0].value] = kw['default_value'].value
    return out


def declared_names(path):
    return {n.args[0].value for n in ast.walk(tree(path))
            if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'DeclareLaunchArgument'}


def strings(path):
    return {n.value for n in ast.walk(tree(path))
            if isinstance(n, ast.Constant) and isinstance(n.value, str)}


def test_graphical_defaults_preserved():
    assert declared_defaults(SYSTEM) == {'gui': 'true', 'use_rviz': 'true', 'debug_window': 'true'}
    assert declared_defaults(SIM) == {'gui': 'true'}
    assert declared_defaults(NAV) == {'use_rviz': 'true'}


def test_options_are_wired_through():
    system = strings(SYSTEM)
    assert {'gui', 'use_rviz', 'debug_window'} <= system
    # the detector's window follows the argument instead of a hard-coded True
    assert "'debug_window': True" not in SYSTEM.read_text()
    nav = NAV.read_text()
    assert "condition=IfCondition(LaunchConfiguration('use_rviz'))" in nav


def test_navigation_reproduces_vendored_turtlebot3_launch():
    ours, theirs = strings(NAV), strings(VENDORED_NAV2)
    for s in ('/bringup_launch.py', 'tb3_navigation2.rviz', 'rviz2', 'map', 'use_sim_time',
              'params_file'):
        assert s in theirs
    assert 'bringup_launch.py' in ours and 'nav2_bringup' in ours
    assert {'tb3_navigation2.rviz', 'rviz2', 'map', 'use_sim_time', 'params_file'} <= ours
    # still the project parameters, map and keep-static BT (Phases 2b and 3)
    assert {'warehouse_map_v2.yaml', 'default_nav_to_pose_bt_xml'} <= ours


def launch_structure(path):
    """Include-argument keys and rviz2 Node keyword/value structure of a launch file."""
    include_keys, rviz = set(), {}
    for n in ast.walk(tree(path)):
        if not isinstance(n, ast.Call):
            continue
        name = getattr(n.func, 'id', '')
        kw = {k.arg: k.value for k in n.keywords}
        if name == 'IncludeLaunchDescription' and 'launch_arguments' in kw:
            d = kw['launch_arguments'].func.value  # {...}.items()
            include_keys |= {k.value for k in d.keys}
        if name == 'Node' and getattr(kw.get('package'), 'value', '') == 'rviz2':
            rviz = {k: (v.value if isinstance(v, ast.Constant) else type(v).__name__)
                    for k, v in kw.items() if k != 'condition'}
    return include_keys, rviz


def test_navigation_matches_vendored_launch_structure():
    """Fails if the vendored TurtleBot3 launch changes its Nav2 arguments or RViz node."""
    ours_inc, ours_rviz = launch_structure(NAV)
    theirs_inc, theirs_rviz = launch_structure(VENDORED_NAV2)
    assert ours_inc == theirs_inc == {'map', 'use_sim_time', 'params_file'}
    assert ours_rviz.keys() == theirs_rviz.keys()
    for k in ('package', 'executable', 'name', 'output'):
        assert ours_rviz[k] == theirs_rviz[k]
    # the vendored file declares exactly these arguments; a new one would need wiring here
    assert declared_names(VENDORED_NAV2) == {'map', 'params_file', 'use_sim_time'}
