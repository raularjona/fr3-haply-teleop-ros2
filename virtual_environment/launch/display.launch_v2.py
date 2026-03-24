from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_ip = LaunchConfiguration('robot_ip')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.11'
        ),

        # TU TELEOP (CLAVE)
        Node(
            package='haply_franka_teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            arguments=[robot_ip]
        ),

        # PUBLICAR CALIBRACIÓN
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/home/hap/handeye_ws/install/easy_handeye2/share/easy_handeye2/launch/publish.launch.py'
            ),
            launch_arguments={
                'name': 'franka_eye_on_base'
            }.items()
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
