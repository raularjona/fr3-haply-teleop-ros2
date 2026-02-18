import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("rviz_config", default_value="moveit.rviz")
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
    # 1. Configuración de MoveIt para FR3
    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="franka_fr3_moveit_config")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("franka_description"), "robots/fr3/fr3.urdf.xacro"))
        .planning_scene_monitor(publish_robot_description=True)
        .to_moveit_configs()
    )

    # 2. Nodo de control principal (Controller Manager)
    # Este nodo es el que gestiona los spawners
    ros2_controllers_path = os.path.join(
        get_package_share_directory("franka_fr3_moveit_config"), "config", "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # 3. LOS SPAWNERS (Los activadores de los controladores)
    
    # El que publica el estado de las articulaciones
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # El que controla el brazo FR3
    # NOTA: Asegúrate de que el nombre coincida con tu ros2_controllers.yaml (suele ser fr3_arm_controller)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "-c", "/controller_manager"],
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 5. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic],
    )

    return [
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        robot_state_publisher,
        rviz_node,
    ]
