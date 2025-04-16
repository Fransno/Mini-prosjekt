#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP

#include "pid_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// ROS2 Node that wraps a PID controller for velocity control, subscribes to /joint_states and publishes to /velocity_controller/commands.
class PIDControllerNode : public rclcpp::Node {
public:
    // Constructor: sets up subscriptions, publishers, and initializes the PID controller
    PIDControllerNode();

private:
    // Callback for joint state messages
    // Computes velocity command based on measured position and velocity
    void joint_states_listener(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Internal PID controller object
    std::unique_ptr<PIDController> pid_controller_;

    // Publishes computed velocity commands to the controller
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;

    // Subscribes to joint state feedback from hardware or simulation
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
};

#endif // PID_CONTROLLER_NODE_HPP
