"""Full simulation stack on a desktop: Gazebo nursery room, map, watchdog, RViz.

    ros2 launch myagv_sprayer_bringup sim_full.launch.py
    ros2 launch myagv_sprayer_bringup sim_full.launch.py scenario:=2_unmapped_persistent
    ros2 launch myagv_sprayer_bringup sim_full.launch.py scenario:=2_unmapped_transient
    ros2 launch myagv_sprayer_bringup sim_full.launch.py localization_mode:=drift

Then, on the same machine or on the Jetson (same ROS_DOMAIN_ID):

    ros2 launch sprayer_bringup jetson.launch.py sim:=true algorithm:=vfh_ql

Scenario handling: the world file always holds the room, the trays and their
ArUco markers. Scenario 2's extra obstacles are spawned at run time from a seed
derived from (scenario, trial), so every algorithm meets the same world -- see
``docs/experiment_protocol.md`` in the Jetson repo for the current scenario
matrix. ``localization_mode:=drift`` is independent of scenario: it injects
localisation noise on top of whichever scenario is running, for ad hoc
robustness experiments -- no scenario in the current protocol turns it on by
itself.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('myagv_sprayer_bringup')
    pkg_gazebo = get_package_share_directory('myagv_sprayer_gazebo')
    pkg_nav = get_package_share_directory('myagv_sprayer_navigation')

    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    scenario = LaunchConfiguration('scenario')
    trial = LaunchConfiguration('trial')

    needs_obstacles = PythonExpression(
        ["'", scenario, "' in ['2_unmapped_persistent', '2_unmapped_transient']"])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='1_ideal',
                              description='1_ideal | 2_unmapped_persistent | '
                                          '2_unmapped_transient'),
        DeclareLaunchArgument('trial', default_value='0'),
        DeclareLaunchArgument('localization_mode', default_value='identity',
                              description="'drift' injects localisation noise "
                                          "independent of scenario"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
            launch_arguments={'gui': gui}.items()),

        Node(package='myagv_sprayer_bringup', executable='camera_stream_node',
             parameters=[os.path.join(pkg_bringup, 'config', 'camera.yaml'),
                         {'source': 'topic', 'use_sim_time': True}],
             output='screen'),

        Node(package='myagv_sprayer_bringup', executable='cmd_vel_watchdog_node',
             name='cmd_vel_watchdog_node', output='screen',
             parameters=[os.path.join(pkg_bringup, 'config', 'watchdog.yaml'),
                         {'use_sim_time': True}]),

        Node(package='myagv_sprayer_bringup', executable='tray_marker_node',
             parameters=[{'use_sim_time': True}], output='screen'),

        # give Gazebo time to spawn the robot before TF consumers start
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'localization.launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'mode': LaunchConfiguration('localization_mode'),
                    'drift_sigma_xy': '0.02',
                    'drift_sigma_theta': '0.02',
                    'initial_offset_m': '0.25',
                }.items()),
        ]),

        TimerAction(period=6.0, actions=[
            Node(package='myagv_sprayer_gazebo', executable='scenario_spawner_node',
                 name='scenario_spawner_node', output='screen',
                 condition=IfCondition(needs_obstacles),
                 parameters=[{'scenario': scenario, 'trial': trial,
                              'use_sim_time': True}]),
        ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'rviz.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(rviz)),
    ])
