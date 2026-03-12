cmake_minimum_required(VERSION 3.8)
project(virtual_environment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

########################################
# Wall node
########################################

add_executable(wall_node
  src/wall_node.cpp
)

ament_target_dependencies(
  wall_node
  rclcpp
  visualization_msgs
  geometry_msgs
)

########################################
# End effector marker node
########################################

add_executable(ee_marker_node
  src/ee_marker_node.cpp
)

ament_target_dependencies(
  ee_marker_node
  rclcpp
  visualization_msgs
  geometry_msgs
)

########################################
# Force feedback node
########################################
add_executable(haptic_wall_node src/haptic_wall_node.cpp)

ament_target_dependencies(
  haptic_wall_node
  rclcpp
  geometry_msgs
)

########################################
# Install executables
########################################

install(TARGETS
  wall_node
  ee_marker_node
  haptic_wall_node
  DESTINATION lib/${PROJECT_NAME}
)

########################################
# Install RViz config
########################################

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
