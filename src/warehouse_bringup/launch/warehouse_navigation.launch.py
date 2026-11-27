from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():

    nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    bringup_pkg = get_package_share_directory('warehouse_bringup')

    map_file = os.path.join(bringup_pkg, 'maps', 'warehouse_map_v2.yaml')

    # Same parameter file the TurtleBot3 launch would pick, with the BT path filled in (Phase 3).
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger_cam')
    params_file = os.path.join(nav2_pkg, 'param', 'humble', model + '.yaml')
    bt_xml = os.path.join(bringup_pkg, 'behavior_trees',
                          'navigate_to_pose_w_replanning_and_recovery_keep_static.xml')
    params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
        convert_types=True)

    # The two actions of turtlebot3_navigation2/launch/navigation2.launch.py (Nav2 bringup and
    # RViz with the TurtleBot3 config), reproduced here so RViz can be switched off (Phase 4b).
    nav2_bringup = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'tb3_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz (false for headless runs)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': params,
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
