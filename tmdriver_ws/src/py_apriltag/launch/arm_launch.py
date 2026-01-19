from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_gripper',
            executable='arm',
            name='arm'
        ),
        Node(
            package='py_apriltag',
            executable='target_pose',
            name='target_pose'
        ),
    ])