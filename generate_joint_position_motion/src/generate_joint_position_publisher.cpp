#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "franka/duration.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joint_state_publisher");

    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("franka_joint_target", 10);

    rclcpp::WallRate loop_rate(1000); // 1 Hz loop rate
    double joint1_value = 0;

    while (rclcpp::ok()) {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.position = {joint1_value, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Joint_1_value{%.2f}",joint1_value);
        publisher->publish(joint_state_msg);
        joint1_value -= 0.001;
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
