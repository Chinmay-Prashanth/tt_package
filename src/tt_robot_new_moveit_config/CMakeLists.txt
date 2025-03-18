cmake_minimum_required(VERSION 3.22)
project(tt_robot_new_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable for the joint state converter node
add_executable(joint_state_converter_node scripts/joint_state_converter.cpp)
ament_target_dependencies(joint_state_converter_node
  rclcpp
  sensor_msgs
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Install the joint state converter node executable
install(TARGETS
  joint_state_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
