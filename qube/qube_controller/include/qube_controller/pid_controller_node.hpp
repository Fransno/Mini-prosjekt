#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP

#include "pid_controller.hpp"    
#include "rclcpp/rclcpp.hpp" // ROS2 C++ client library
#include "sensor_msgs/msg/joint_state.hpp" 
#include "std_msgs/msg/float64_multi_array.hpp" 

class PIDControllerNode : public rclcpp::Node {
public:
    // Constructor: Initializes the Node, declares parameters, and sets up subscriptions and publications
    PIDControllerNode();

private:
    // Callback function for handling incoming joint state messages
    void joint_states_listener(const sensor_msgs::msg::JointState::SharedPtr msg);

    // PID controller instance for computing the velocity command
    std::unique_ptr<PIDController> pid_controller_;

    // Publisher for sending computed velocity commands
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;

    // Subscriber for receiving joint states (e.g., position and velocity) from a robot or simulation
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
};

#endif // PID_CONTROLLER_NODE_HPP

