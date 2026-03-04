cmake_minimum_required(VERSION 3.8)
project(haply_franka_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Franka REQUIRED)
find_package(sensor_msgs REQUIRED)

#add_executable(teleop_node franka_haply_node.cpp)
add_executable(teleop_node teleop_node.cpp)

# He quitado la palabra 'PRIVATE' para que no choque con la línea de abajo
target_link_libraries(teleop_node Franka::Franka)
ament_target_dependencies(teleop_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS teleop_node DESTINATION lib/${PROJECT_NAME})
ament_package()
