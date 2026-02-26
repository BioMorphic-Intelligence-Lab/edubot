import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set the path to this package.
    pkg_share = FindPackageShare(package='lerobot').find('lerobot')

    # Set the path to the URDF file
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/lerobot.urdf')

    with open(default_urdf_model_path, 'r') as infp:
        robot_desc = infp.read()

    # Launch configuration variables
    urdf_model = LaunchConfiguration('urdf_model', default=default_urdf_model_path)
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub', default='True')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # Declare the launch arguments
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    # Publish the joint state values for the non-fixed joints in the URDF file.
    # (joint_state_publisher_gui publishes joint states; we only need the GUI here)
    start_joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')

    # Subscribe to the joint states and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_desc}],
        arguments=[default_urdf_model_path])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
