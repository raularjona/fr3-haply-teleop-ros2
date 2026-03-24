from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    robot_ip = LaunchConfiguration('robot_ip')

    pkg_share = FindPackageShare('virtual_environment').find('virtual_environment')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    return LaunchDescription([

        # Argumento configurable
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.11'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_file).read()
            }],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),

        # Teleop
        Node(
            package='haply_franka_teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            arguments=[robot_ip]
        )
    ])
