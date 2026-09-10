import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory('myagv_sprayer_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(desc, 'rviz', 'myagv_sprayer.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
