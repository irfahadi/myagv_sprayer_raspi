"""Online SLAM (slam_toolbox) to build a fresh map of the nursery room.

    ros2 launch myagv_sprayer_navigation slam.launch.py use_sim_time:=true
    # drive around with teleop, then:
    ros2 run nav2_map_server map_saver_cli -f maps/nursery_room_scan
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('myagv_sprayer_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = os.path.join(nav_dir, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),
    ])
