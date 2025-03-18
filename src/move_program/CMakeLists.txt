cmake_minimum_required(VERSION 3.8)
project(move_program)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

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

# Install all executables (including tt_motion_rl_1)
install(TARGETS move_program move_random tt_motion tt_motion_hit tt_motion_rl tt_motion_rl_1
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
