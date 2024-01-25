#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <array>
#include <iterator>

#include "franka/robot.h"
#include "franka/model.h"
#include "franka/exception.h"
#include "franka/duration.h"

#include "rclcpp/rclcpp.hpp"
#include "generate_joint_position_motion/common.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
    ostream << "[";
    for (size_t i = 0; i < N; ++i) {
        ostream << array[i];
        if (i < N - 1) ostream << ", ";
    }
    ostream << "]";
    return ostream;
}

class FrankaController : public rclcpp::Node {
public:
    FrankaController() : Node("franka_controller") {
        subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "franka_pose_target", 10, 
            std::bind(&FrankaController::poseStateCallback, this, std::placeholders::_1));
    }

    std::array<double, 7> getPoseValues() {
        std::lock_guard<std::mutex> lock(mutex_);
        // RCLCPP_INFO(this->get_logger(), "Pose received: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        //             pose_positions_[0], pose_positions_[1], pose_positions_[2],
        //             pose_positions_[3], pose_positions_[4], pose_positions_[5],
        //             pose_positions_[6]);
        RCLCPP_INFO(this->get_logger(), "Pose received: [%.2f, %.2f]",
                    pose_positions_[0], pose_positions_[2]);
        return pose_positions_;
    }

private:
    void poseStateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        pose_positions_[0] = msg->pose.position.x;
        pose_positions_[1] = msg->pose.position.y;
        pose_positions_[2] = msg->pose.position.z;
        pose_positions_[3] = msg->pose.orientation.w;
        pose_positions_[4] = msg->pose.orientation.x;
        pose_positions_[5] = msg->pose.orientation.y;
        pose_positions_[6] = msg->pose.orientation.z;
    }

    std::array<double, 7> pose_positions_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
    std::mutex mutex_;
};

void controlRobot(franka::Robot& robot, std::shared_ptr<FrankaController> controller) {
    try {
        double time = 0.0;
        std::array<double, 16> initial_pose;
        std::array<double, 7> target_pose;

        robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
            if (time == 0.0) {
                initial_pose = robot_state.O_T_EE_c;
                // target_pose = controller->getPoseValues();
            }
            time += period.toSec();
            target_pose = controller->getPoseValues();
            // constexpr double kRadius = 0.3;
            // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
            // double delta_x = kRadius * std::sin(angle);
            // double delta_z = kRadius * (std::cos(angle) - 1);

            std::array<double, 16> new_pose = initial_pose;
            // new_pose[12] += delta_x;
            // new_pose[14] += delta_z;
            new_pose[12] += target_pose[0];
            new_pose[14] += target_pose[2];
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose received: [%.2f, %.2f] at time: %.6f",new_pose[12], new_pose[14],time);

            return new_pose;
        });
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    std::string robot_ip = argv[1];

    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.1, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    auto controller = std::make_shared<FrankaController>();
    std::thread robot_control_thread([&]() { controlRobot(robot, controller); });

    rclcpp::spin(controller);

    robot_control_thread.join();
    rclcpp::shutdown();

    return 0;
}
