"""Real-robot bringup on the myAGV Raspberry Pi.

This side of the system is now deliberately thin: sensors in, wheels out. Every
navigation decision -- VFH-QL, CQL, SARSA, A* or D* Lite -- is made on the
Jetson Nano (``ros2 launch sprayer_bringup jetson.launch.py``), so what runs
here is

  * the myAGV base driver          -> /odom, TF odom->base_footprint
  * the YDLidar driver             -> /scan
  * camera_stream_node             -> /camera/image_raw/compressed (over the LAN)
  * map_server + localization      -> /map, TF map->odom   (for A* / D* Lite only)
  * cmd_vel_watchdog_node          -> /cmd_vel, the ONLY publisher on that topic
  * robot_health_node, tray_marker_node

The watchdog is the reason this split is safe: if the Jetson link drops, the
wheels stop within ``timeout_s`` instead of coasting on the last command.

    ros2 launch myagv_sprayer_bringup hardware.launch.py
    ros2 launch myagv_sprayer_bringup hardware.launch.py localization:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('myagv_sprayer_bringup')
    pkg_desc = get_package_share_directory('myagv_sprayer_description')
    pkg_nav = get_package_share_directory('myagv_sprayer_navigation')

    localization = LaunchConfiguration('localization')
    myagv_driver_launch = LaunchConfiguration('myagv_driver_launch')
    lidar_launch = LaunchConfiguration('lidar_launch')

    urdf = os.path.join(pkg_desc, 'urdf', 'myagv_sprayer.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' use_sim:=false']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('localization_mode', default_value='slam',
                              description='slam | identity | drift | off'),
        DeclareLaunchArgument('myagv_driver_launch',
                              default_value='myagv_odometry myagv_active.launch.py'),
        DeclareLaunchArgument('lidar_launch',
                              default_value='ydlidar_ros2_driver ydlidar_launch.py'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': False}], output='screen'),

        # Elephant Robotics driver: subscribes /cmd_vel, publishes /odom + TF
        ExecuteProcess(cmd=['ros2', 'launch', myagv_driver_launch],
                       shell=True, output='screen'),

        ExecuteProcess(cmd=['ros2', 'launch', lidar_launch,
                            'params_file:=' + os.path.join(pkg_bringup, 'config',
                                                           'ydlidar.yaml')],
                       shell=True, output='screen'),

        # the only publisher on /cmd_vel
        Node(package='myagv_sprayer_bringup', executable='cmd_vel_watchdog_node',
             name='cmd_vel_watchdog_node', output='screen',
             parameters=[os.path.join(pkg_bringup, 'config', 'watchdog.yaml')]),

        Node(package='myagv_sprayer_bringup', executable='camera_stream_node',
             parameters=[os.path.join(pkg_bringup, 'config', 'camera.yaml')],
             output='screen'),

        Node(package='myagv_sprayer_bringup', executable='robot_health_node',
             output='screen'),

        Node(package='myagv_sprayer_bringup', executable='tray_marker_node',
             output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'localization.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'mode': LaunchConfiguration('localization_mode'),
            }.items(),
            condition=IfCondition(localization)),
    ])
