from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tm_driver',
            executable='tm_driver',
            name='tm_driver_node',
            arguments=['192.168.10.2']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robotiq_85_driver'),
                'launch',
                'gripper_driver.launch.py'
            )),
        ),
        Node(
            package='py_gripper',
            executable='arm_feedback_states',
            name='arm_feedback_states'
        ),
    ])