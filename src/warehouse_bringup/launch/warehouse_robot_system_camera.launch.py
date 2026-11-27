from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_pkg = get_package_share_directory('warehouse_bringup')

    gazebo_robot_launch = os.path.join(
        bringup_pkg, 'launch', 'warehouse_simulation_camera.launch.py'
    )

    nav2_launch = os.path.join(
        bringup_pkg, 'launch', 'warehouse_navigation.launch.py'
    )

    gazebo_and_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_robot_launch),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    nav2_delayed = TimerAction(
        period=40.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()
            )
        ]
    )

    aruco_detector = TimerAction(
        period=55.0,
        actions=[
            Node(
                package='warehouse_perception',
                executable='aruco_station_detector',
                name='aruco_station_detector',
                parameters=[{'use_sim_time': True,
                             'debug_window': ParameterValue(
                                 LaunchConfiguration('debug_window'), value_type=bool)}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Graphical defaults for development and demonstrations. Headless (Phase 4b):
        # gui:=false use_rviz:=false debug_window:=false
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo client (false runs gzserver only)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'debug_window', default_value='true',
            description='Show the ArUco detector OpenCV window'),
        gazebo_and_robot,
        nav2_delayed,
        aruco_detector
    ])
