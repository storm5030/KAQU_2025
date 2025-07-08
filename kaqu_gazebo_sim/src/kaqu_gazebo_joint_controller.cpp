#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// using namespace std::chrono_literals;
using std::placeholders::_1;

class KaQuGazeboJointCtrl : public rclcpp::Node
{   
public:
    KaQuGazeboJointCtrl()
        : Node("kaqu_gazebo_joint_ctrl_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/Kaqu_Joint", 10, std::bind(&KaQuGazeboJointCtrl::callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gazebo_joint_controller/commands", 10);
    }

private:
    void callback(const sensor_msgs::msg::JointState::SharedPtr msg_rx) const
    {   
        // Create Float64MultiArray message for publishing
        auto joint_angles = std_msgs::msg::Float64MultiArray();
        joint_angles.data.reserve(msg_rx->position.size());  // Pre-allocate space

        for (double ang : msg_rx->position) {
            // Convert angles from degrees to radians (if necessary)
            joint_angles.data.push_back(ang * M_PI / 180.0);
        }

        publisher_->publish(joint_angles);
    } 

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;    
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KaQuGazeboJointCtrl>());
    rclcpp::shutdown();
    return 0;
}
