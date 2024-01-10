#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::cin;
using std::cout;
using std::endl;
using std::getline;
using std::shared_ptr;
using std::string;
using std::vector;

class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher() : Node("joint_state_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("franka_joint_target", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&JointStatePublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        cout << "Enter joint positions (comma-separated): ";
        string input_str;
        getline(cin, input_str);

        auto message = sensor_msgs::msg::JointState();
        message.position = parse_joint_positions(input_str);
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing joint positions");
    }

    vector<double> parse_joint_positions(const string& input) {
        vector<double> positions;
        size_t start;
        size_t end = 0;
        while ((start = input.find_first_not_of(',', end)) != string::npos) {
            end = input.find(',', start);
            positions.push_back(stod(input.substr(start, end - start)));
        }
        return positions;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    shared_ptr<JointStatePublisher> node = std::make_shared<JointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
