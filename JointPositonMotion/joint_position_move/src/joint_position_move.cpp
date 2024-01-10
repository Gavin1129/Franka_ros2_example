#include <iostream>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "joint_position_move/common.h"

class FrankaController : public rclcpp::Node {
public:
    FrankaController() : Node("franka_controller") {
        // Initialize subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "franka_joint_target", 10, std::bind(&FrankaController::jointStateCallback, this, std::placeholders::_1));
        try {
            // Connect to robot
            robot_ = std::make_unique<franka::Robot>("172.16.0.2"); // Replace with your robot's IP address
            // Set robot to initial state, etc.
        } catch (const franka::Exception& e) {
            std::cerr << e.what() << std::endl;
            // Handle exception appropriately
        }
    }
private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        try {
            // Extract joint positions
            std::array<double, 7> joint_positions;
            if (msg->position.size() >= 7) {
                RCLCPP_INFO(this->get_logger(),"Position:(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)", 
                        msg->position[0],msg->position[1],msg->position[2],msg->position[3],
                        msg->position[4],msg->position[5],msg->position[6]);
                std::copy(msg->position.begin(), msg->position.begin() + 7, joint_positions.begin());
                // Move robot
                MotionGenerator motion_generator(0.1,joint_positions);
                robot_->control(motion_generator); // This is a simplified example. You should use a motion generator.
            } else {
                RCLCPP_ERROR(this->get_logger(), "Received joint state with insufficient data.");
            }
        } catch (const franka::Exception& e) {
            std::cerr << e.what() << std::endl;
            // Handle exception appropriately
        }
    }
    std::unique_ptr<franka::Robot> robot_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
