from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():

    nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    bringup_pkg = get_package_share_directory('warehouse_bringup')

    map_file = os.path.join(bringup_pkg, 'maps', 'warehouse_map_v2.yaml')

    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')

    # Same parameter file the TurtleBot3 launch would pick, with the BT path filled in (Phase 3).
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger_cam')
    params_file = os.path.join(nav2_pkg, 'param', 'humble', model + '.yaml')
    bt_xml = os.path.join(bringup_pkg, 'behavior_trees',
                          'navigate_to_pose_w_replanning_and_recovery_keep_static.xml')
    params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
        convert_types=True)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': params,
            }.items()
        )
    ])
