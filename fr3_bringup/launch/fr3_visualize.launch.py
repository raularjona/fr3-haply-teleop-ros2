from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue # <--- Necesario para el fix

def generate_launch_description():
    # 1. Obtener la descripción del robot (URDF/Xacro)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "fr3", "fr3.urdf.xacro"]),
    ])

    # 2. ENVOLVER EN PARAMETERVALUE (Aquí está el arreglo del error)
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 3. Nodo que publica el estado del robot
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 4. Nodo RViz2 (sin archivo de configuración por ahora para que no falle si no existe)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription([
        robot_state_pub_node,
        rviz_node,
    ])
