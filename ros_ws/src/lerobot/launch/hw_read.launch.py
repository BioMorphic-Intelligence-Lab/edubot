from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('lerobot')
    param_file = os.path.join(package_dir, 'config', 'robot_read.yaml')

    return LaunchDescription([
        Node(
            package='lerobot',
            executable='lerobot_read',
            name='lerobot_read',
            parameters=[param_file],
            output='screen',
        ),
    ])
