"""Gazebo Classic simulation of the myAGV sprayer inside the nursery room.

    ros2 launch myagv_sprayer_gazebo sim.launch.py            # world + robot
    ros2 launch myagv_sprayer_gazebo sim.launch.py gui:=false # headless

The robot spawns just inside the main entrance (south wall), facing north.
Adapted from beam_agrobot_v2 robot_description/launch/gazebo.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('myagv_sprayer_gazebo')
    pkg_desc = get_package_share_directory('myagv_sprayer_description')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    urdf = os.path.join(pkg_desc, 'urdf', 'myagv_sprayer.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' use_sim:=true']), value_type=str)

    # Make model:// URIs in the world resolve to this package's models/
    model_path = os.path.join(pkg_gazebo, 'models')
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path + (':' + existing if existing else ''))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'false'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'myagv_sprayer',
                   '-topic', 'robot_description',
                   '-x', x, '-y', y, '-z', '0.05', '-Y', yaw],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(
            pkg_gazebo, 'worlds', 'nursery_room.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-2.3'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        set_model_path,
        gzserver,
        gzclient,
        rsp,
        spawn,
    ])
