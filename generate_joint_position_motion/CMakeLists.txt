cmake_minimum_required(VERSION 3.8)
project(generate_joint_position_motion)

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

add_executable(move_to_pose src/move_to_pose.cpp src/common.cpp)
add_executable(generate_joint_position_publisher src/generate_joint_position_publisher.cpp src/common.cpp)
add_executable(move_to_cartesian_pose src/move_to_cartesian_pose.cpp src/common.cpp)
add_executable(generate_pose_position_publisher src/generate_pose_position_publisher.cpp src/common.cpp)


target_include_directories(move_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(generate_joint_position_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)  

target_include_directories(move_to_cartesian_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
) 

target_include_directories(generate_pose_position_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(generate_joint_position_publisher franka)
target_link_libraries(move_to_pose franka)
target_link_libraries(move_to_cartesian_pose franka)
target_link_libraries(generate_pose_position_publisher franka)

target_compile_features(move_to_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_pose
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

ament_target_dependencies(generate_joint_position_publisher
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

ament_target_dependencies(move_to_cartesian_pose
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

ament_target_dependencies(generate_pose_position_publisher
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

install(TARGETS move_to_pose
  generate_joint_position_publisher
  move_to_cartesian_pose
  generate_pose_position_publisher
  DESTINATION lib/${PROJECT_NAME})

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
