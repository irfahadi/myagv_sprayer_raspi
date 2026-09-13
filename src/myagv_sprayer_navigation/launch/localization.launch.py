"""Map + map->odom, without Nav2.

    ros2 launch myagv_sprayer_navigation localization.launch.py            # hardware, slam_toolbox
    ros2 launch myagv_sprayer_navigation localization.launch.py mode:=identity use_sim_time:=true
    ros2 launch myagv_sprayer_navigation localization.launch.py mode:=drift  \
        drift_sigma_xy:=0.02 initial_offset_m:=0.25        # Scenario B

Only A* and D* Lite consume any of this. The RL policy is deliberately blind to it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('myagv_sprayer_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map')

    use_slam = PythonExpression(["'", mode, "' == 'slam'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='slam',
                              description='slam | identity | drift | off'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            pkg, 'maps', 'nursery_room.yaml')),
        DeclareLaunchArgument('drift_sigma_xy', default_value='0.010'),
        DeclareLaunchArgument('drift_sigma_theta', default_value='0.010'),
        DeclareLaunchArgument('initial_offset_m', default_value='0.0'),
        # Named 'localization_seed', not 'seed': gazebo_ros's gzserver.launch.py
        # already declares a 'seed' argument (default_value='') for the physics
        # RNG. Nested IncludeLaunchDescriptions share one flat LaunchConfiguration
        # namespace, and DeclareLaunchArgument never overrides an
        # already-declared value -- so a same-named 'seed' here would silently
        # inherit gzserver's empty default instead of applying '0', and
        # localization_node would crash on int('').
        DeclareLaunchArgument('localization_seed', default_value='0'),

        Node(package='myagv_sprayer_navigation', executable='map_server_node',
             name='map_server_node', output='screen',
             parameters=[{'map_yaml': map_yaml, 'use_sim_time': use_sim_time}]),

        Node(package='myagv_sprayer_navigation', executable='localization_node',
             name='localization_node', output='screen',
             condition=UnlessCondition(use_slam),
             respawn=True, respawn_delay=1.0,
             parameters=[{
                 'mode': mode, 'use_sim_time': use_sim_time,
                 'drift_sigma_xy': LaunchConfiguration('drift_sigma_xy'),
                 'drift_sigma_theta': LaunchConfiguration('drift_sigma_theta'),
                 'initial_offset_m': LaunchConfiguration('initial_offset_m'),
                 'localization_seed': LaunchConfiguration('localization_seed'),
             }]),

        Node(package='slam_toolbox', executable='localization_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             condition=IfCondition(use_slam),
             parameters=[os.path.join(pkg, 'config', 'slam_toolbox_params.yaml'),
                         {'use_sim_time': use_sim_time,
                          'map_file_name': os.path.join(pkg, 'maps', 'nursery_room'),
                          'mode': 'localization'}]),
    ])
