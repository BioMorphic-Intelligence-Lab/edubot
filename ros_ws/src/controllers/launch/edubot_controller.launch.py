import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the parameter file
    param_file = os.path.join(get_package_share_directory('controllers'), 'config', 'edubot_params.yaml')

    return LaunchDescription([
        Node(
            package='controllers',
            executable='example_traj',
            name='example_raj_edubot',
            output='screen',
            parameters=[param_file]
        )
    ])
