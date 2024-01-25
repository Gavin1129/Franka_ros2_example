#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "franka/duration.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pose_state_publisher");

    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("franka_pose_target", 10);

    rclcpp::WallRate loop_rate(1000); // 1 Hz loop rate
    geometry_msgs::msg::PoseStamped new_pose;
    double time = 0;
    franka::Duration period;

    while (rclcpp::ok()) {
        // std::cout << period. << std::endl;
        constexpr double kRadius = 0.01;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 5 * time));
        double delta_x = kRadius * std::sin(angle);
        double delta_z = kRadius * (std::cos(angle) - 1);
        new_pose.pose.position.x = delta_x;
        new_pose.pose.position.z = delta_z;

        time += 0.001;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"pose:{%.2f,0,%.2f}",delta_x,delta_z);
        publisher->publish(new_pose);
        rclcpp::spin_some(node);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
