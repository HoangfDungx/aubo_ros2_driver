cmake_minimum_required(VERSION 3.22)
project(aubo_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(aubo_msgs REQUIRED)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs" "moveit_msgs" "aubo_msgs" "moveit_ros_planning_interface")

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(moveit_executor_node src/moveit_executor_node.cpp)
ament_target_dependencies(moveit_executor_node
  ${dependencies}
)

target_link_libraries(moveit_executor_node 
  ${ament_LIBRARIES}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
