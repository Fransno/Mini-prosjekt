#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP

#include "pid_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PIDControllerNode : public rclcpp::Node {
public:
    PIDControllerNode();

private:
    void joint_states_listener(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    std::unique_ptr<PIDController> pid_controller_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
};

#endif // PID_CONTROLLER_NODE_HPP