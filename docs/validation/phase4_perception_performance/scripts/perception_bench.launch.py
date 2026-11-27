"""Perception bench: warehouse world (Gazebo GUI as in the mission system) + one camera-model variant at a pose."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wg = get_package_share_directory('warehouse_gazebo'); tg = get_package_share_directory('turtlebot3_gazebo')
    here = os.path.dirname(os.path.realpath(__file__))
    models = os.path.join(here, '..', 'models')
    paths = '/usr/share/gazebo-11/models:' + os.path.join(wg, 'models') + ':' + os.path.join(tg, 'models') + ':' + models
    a = {k: LaunchConfiguration(k) for k in ('model', 'x', 'y', 'yaw', 'gui')}
    world = os.path.join(here, 'small_warehouse_state.world')   # world + gazebo_ros_state (bench only)
    plug = ['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world] + plug, output='screen', condition=IfCondition(a['gui']))
    server = ExecuteProcess(cmd=['gzserver', '--verbose', world] + plug, output='screen', condition=UnlessCondition(a['gui']))
    spawn = TimerAction(period=6.0, actions=[Node(package='gazebo_ros', executable='spawn_entity.py', output='screen', arguments=[
        '-entity', 'robot', '-file', [models, '/turtlebot3_', a['model'], '/model.sdf'],
        '-x', a['x'], '-y', a['y'], '-z', '0.05', '-Y', a['yaw']])])
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='cam_wide182_30hz'), DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'), DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', paths), gazebo, server, spawn])
