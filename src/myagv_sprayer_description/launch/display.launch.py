"""Show the myAGV sprayer URDF in RViz with a joint_state_publisher GUI."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('myagv_sprayer_description')
    urdf = os.path.join(pkg, 'urdf', 'myagv_sprayer.urdf.xacro')
    rviz_cfg = os.path.join(pkg, 'rviz', 'myagv_sprayer.rviz')

    use_gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' use_sim:=false']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ])
