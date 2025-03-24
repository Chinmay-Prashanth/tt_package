cmake_minimum_required(VERSION 3.8)
project(tt_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Python
find_package(Python3 COMPONENTS Development REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${Python3_INCLUDE_DIRS}
)

# Create the tt_robot_controller executable
add_executable(tt_robot_controller src/tt_robot_controller.cpp)
target_include_directories(tt_robot_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tt_robot_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(tt_robot_controller
  rclcpp
  sensor_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)
target_link_libraries(tt_robot_controller
  ${Python3_LIBRARIES}
)

# Existing executable: move_program
add_executable(move_program src/move_program.cpp)
target_include_directories(move_program PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_program PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  move_program
  "moveit_ros_planning_interface"
  "rclcpp"
)

# New executable: move_random
add_executable(move_random src/move_random.cpp)
target_include_directories(move_random PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_random PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  move_random
  "moveit_ros_planning_interface"
  "rclcpp"
)

# New executable: rally_simulator
add_executable(rally_simulator src/rally_simulator.cpp)
target_include_directories(rally_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rally_simulator PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  rally_simulator
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

# New executable: tt_motion
add_executable(tt_motion src/tt_motion.cpp)
target_include_directories(tt_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tt_motion PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  tt_motion
  "moveit_ros_planning_interface"
  "rclcpp"
)

# New executable: tt_motion_hit
add_executable(tt_motion_hit src/tt_motion_hit.cpp)
target_include_directories(tt_motion_hit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tt_motion_hit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  tt_motion_hit
  "moveit_ros_planning_interface"
  "rclcpp"
)

# New executable: tt_motion_rl
add_executable(tt_motion_rl src/tt_motion_rl.cpp)
target_include_directories(tt_motion_rl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tt_motion_rl PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  tt_motion_rl
  "moveit_ros_planning_interface"
  "rclcpp"
)

# New executable: tt_motion_rl_1
add_executable(tt_motion_rl_1 src/tt_motion_rl_1.cpp)
target_include_directories(tt_motion_rl_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tt_motion_rl_1 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  tt_motion_rl_1
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Install targets
install(TARGETS
  tt_robot_controller
  move_program
  move_random
  rally_simulator
  tt_motion
  tt_motion_hit
  tt_motion_rl
  tt_motion_rl_1
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  src/SMS_STS
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/nema.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
