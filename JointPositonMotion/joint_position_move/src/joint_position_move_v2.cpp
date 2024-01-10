#include <iostream>
#include <array>
#include <cmath>
#include <memory>
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
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "franka_joint_target", 10, std::bind(&FrankaController::jointStateCallback, this, std::placeholders::_1));

        try {
            robot_ = std::make_unique<franka::Robot>("172.16.0.2");
            current_joint_positions_.fill(0.0);
            target_joint_positions_.fill(0.0);
        } catch (const franka::Exception& e) {
            std::cerr << e.what() << std::endl;
            rclcpp::shutdown();
        }
    }

    void run() {
        while (rclcpp::ok()) {
            checkTargetReached();
            rclcpp::spin_some(shared_from_this());
        }
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        try {
            if (msg->position.size() >= 7) {
                RCLCPP_INFO(this->get_logger(), "Received Joint State");
                std::copy(msg->position.begin(), msg->position.begin() + 7, target_joint_positions_.begin());
                MotionGenerator motion_generator(0.1, target_joint_positions_);
                robot_->control(motion_generator);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Received joint state with insufficient data.");
            }
        } catch (const franka::Exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }

    void checkTargetReached() {
        try {
            auto robot_state = robot_->readOnce();
            std::copy(robot_state.q.begin(), robot_state.q.end(), current_joint_positions_.begin());

            if (isCloseEnough(current_joint_positions_, target_joint_positions_)) {
                target_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "Target position reached. Shutting down node.");
                rclcpp::shutdown();
            }
        } catch (const franka::Exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }

    bool isCloseEnough(const std::array<double, 7>& current, const std::array<double, 7>& target) {
        const double threshold = 0.01; // Define a suitable threshold
        for (size_t i = 0; i < current.size(); ++i) {
            if (std::abs(current[i] - target[i]) > threshold) {
                return false;
            }
        }
        return true;
    }

    std::unique_ptr<franka::Robot> robot_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    std::array<double, 7> current_joint_positions_;
    std::array<double, 7> target_joint_positions_;
    bool target_reached_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaController>();
    node->run();
    return 0;
}
