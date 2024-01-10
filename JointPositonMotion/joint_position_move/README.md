# Franka joint postion motion example

Task:
1. Write a publisher node to publish target positon for Franka robot arm. 
2. Write a subscription node to receive the position message and contol the arm move to that pose.

## Package creation
```
ros2 pkg create joint_position_move --build-type ament_cmake --dependencies rclcpp std_msgs sensor_msgs --node-name joint_position_move
```
## CMakeLists.txt
This node need libfranka included
```
# Add include directories for Eigen3 and libfranka
include_directories(
  include
  /usr/include/eigen3  # Eigen3 include directory
  /home/carlmoore/libfranka/include  # libfranka include directory
)

add_executable(joint_position_move src/joint_position_move.cpp src/common.cpp)

target_link_libraries(joint_position_move franka)

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
```

## Subscription node
There two version of subscription node

1. Receive the message from topic "franka_joint_target" continually  
```
ros2 run joint_position_move joint_position_move
```
2. Receive the message from topic "franka_joint_target". It will control the arm move to the target pose and then shutdown 
```
ros2 run joint_position_move joint_position_move_v2
```
## Publication node
There are two version of publisher node
1. Publish a predefined message. The target position is defined in the code. 
```
ros2 run joint_positon_move joint_position_publisher
```
2. Continually read input from terminal and then publish 
```
ros2 run joint_positon_move joint_position_publisher_read_terminal
```
output:
```
Enter joint positions (comma-separated):
```
Then you can input the target value like this:

```
Enter joint positions (comma-separated): 0,0,0,0,0,0,0
```
