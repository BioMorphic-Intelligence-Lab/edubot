from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('lerobot')
    
    # Path to parameter file
    param_file = os.path.join(package_dir, 'config', 'pink_robot.yaml')
    
    return LaunchDescription([
        Node(
            package='lerobot',
            executable='lerobot_hw',
            name='lerobot',
            parameters=[param_file],
            output='screen'
        )
    ])