from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    warehouse_gazebo_pkg = get_package_share_directory('warehouse_gazebo')
    turtlebot3_description_pkg = get_package_share_directory('turtlebot3_description')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(
        warehouse_gazebo_pkg,
        'worlds',
        'small_warehouse.world'
    )

    # Camera-enabled TurtleBot3 model: project copy of turtlebot3_burger_cam with a 100 degree
    # pinhole camera instead of the 182 degree wide-angle sensor (v1.1 Phase 4; the wide-angle
    # sensor's cube-map rendering delivered ~1 Hz on a software renderer).
    robot_sdf = os.path.join(
        warehouse_gazebo_pkg, 'models', 'turtlebot3_burger_cam_pinhole', 'model.sdf'
    )

    # Stable URDF
    robot_urdf = os.path.join(
        turtlebot3_description_pkg,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    gazebo_models = (
        '/usr/share/gazebo-11/models:'
        + (os.path.join(warehouse_gazebo_pkg, 'models') + ':')
        + os.path.join(turtlebot3_gazebo_pkg, 'models')
    )

    gui = LaunchConfiguration('gui')
    server_args = [world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']

    # gui:=true (default): gzserver + gzclient, as before. gui:=false: gzserver only, which frees
    # the CPU the client spends rendering the scene a second time.
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose'] + server_args,
        output='screen',
        condition=IfCondition(gui)
    )
    gazebo_server_only = ExecuteProcess(
        cmd=['gzserver', '--verbose'] + server_args,
        output='screen',
        condition=UnlessCondition(gui)
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                robot_urdf,
                ' namespace:=""'
            ]),
            'use_sim_time': True
        }],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3',
                    '-file', robot_sdf,
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.05'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo client (false runs gzserver only)'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger_cam'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_models),

        gazebo,
        gazebo_server_only,
        robot_state_publisher,
        spawn_robot
    ])
