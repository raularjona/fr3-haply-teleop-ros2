import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. URDF y SRDF (con ParameterValue para evitar errores de parseo)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "fr3", "fr3.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "fr3", "fr3.srdf.xacro"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # 2. Cargar YAMLs de Franka como diccionarios (esto evita el error de ros__parameters)
    kinematics_yaml = load_yaml("franka_description", "robots/fr3/kinematics.yaml")
    joint_limits_yaml = load_yaml("franka_description", "robots/fr3/joint_limits.yaml")

    # 3. Cargar tu configuración de Servo (esta sí debería tener ros__parameters, pero la cargamos igual)
    servo_yaml_path = os.path.join(get_package_share_directory("fr3_control"), "config", "fr3_servo_config.yaml")
    with open(servo_yaml_path, 'r') as f:
        servo_config = yaml.safe_load(f)
    
    # Extraemos solo la parte de parámetros si el YAML tiene la estructura de nodo
    if 'servo_node' in servo_config:
        servo_params = servo_config['servo_node']['ros__parameters']
    else:
        servo_params = servo_config

    # 4. Nodo de MoveIt Servo
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            servo_params,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])
