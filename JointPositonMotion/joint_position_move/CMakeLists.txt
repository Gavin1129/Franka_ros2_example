cmake_minimum_required(VERSION 3.8)
project(joint_position_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add include directories for Eigen3 and libfranka
include_directories(
  include
  /usr/include/eigen3  # Eigen3 include directory
  /home/carlmoore/libfranka/include  # libfranka include directory
)

add_executable(joint_position_move src/joint_position_move.cpp src/common.cpp)
add_executable(joint_position_move_v2 src/joint_position_move_v2.cpp src/common.cpp)
add_executable(joint_position_publisher src/joint_position_publisher.cpp)
add_executable(joint_position_publisher_read_terminal src/joint_position_publisher_read_terminal.cpp)

target_include_directories(joint_position_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joint_position_move PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(joint_position_move franka)
target_link_libraries(joint_position_move_v2 franka)

ament_target_dependencies(
  joint_position_move
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
ament_target_dependencies(
  joint_position_move_v2
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

ament_target_dependencies(
  joint_position_publisher
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
ament_target_dependencies(
  joint_position_publisher_read_terminal
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

install(TARGETS joint_position_move
  joint_position_move_v2
  joint_position_publisher
  joint_position_publisher_read_terminal
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
