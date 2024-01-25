#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

#include "franka/robot.h"
#include "franka/duration.h"
#include "franka/exception.h"
#include <franka/model.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "generate_joint_position_motion/common.h"

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

class FrankaController : public rclcpp::Node {
public:
    FrankaController() : Node("franka_controller") {
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "franka_joint_target", 10, 
            std::bind(&FrankaController::jointStateCallback, this, std::placeholders::_1));
    }

    std::array<double, 7> getJointValues() {
        std::lock_guard<std::mutex> lock(mutex_);
        return joint_positions_;
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->position.size() >= 7) {
            std::copy(msg->position.begin(), msg->position.begin() + 7, joint_positions_.begin());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Received joint state with insufficient data.");
        }
    }

    std::array<double, 7> joint_positions_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    std::mutex mutex_;
};

void controlRobot(const std::string& robot_ip, double speed_factor, std::shared_ptr<FrankaController> controller) {
    try {
        franka::Robot robot(robot_ip);
        franka::Model model(robot.loadModel());
        setDefaultBehavior(robot);

        while (rclcpp::ok()) {
            auto q_goal = controller->getJointValues();
            MotionGenerator motion_generator(speed_factor, q_goal);
            franka::RobotState state = robot.readOnce();
            for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector;
                 frame++)
            {
                std::cout << model.pose(frame, state) << std::endl;
            }
            // std::cout << "Press Enter to continue..." << std::endl;
            // std::cin.ignore();

            robot.control(motion_generator);
        }
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
                  << "<speed-factor>" << std::endl
                  << "joint0 to joint6 are joint angles in [rad]." << std::endl
                  << "speed-factor must be between zero and one." << std::endl;
        return -1;
    }

    std::string robot_ip = argv[1];
    // double speed_factor = std::stod(argv[2]);
    double speed_factor = 0.8;

    auto controller = std::make_shared<FrankaController>();
    std::thread robot_control_thread(controlRobot, robot_ip, speed_factor, controller);

    rclcpp::spin(controller);

    robot_control_thread.join();
    rclcpp::shutdown();

    return 0;
}
