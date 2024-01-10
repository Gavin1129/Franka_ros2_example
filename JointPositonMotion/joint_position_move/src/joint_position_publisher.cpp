#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joint_state_publisher");

    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("franka_joint_target", 10);

    rclcpp::WallRate loop_rate(1); // 1 Hz loop rate

    while (rclcpp::ok()) {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.position = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};  // Example joint positions
        publisher->publish(joint_state_msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
