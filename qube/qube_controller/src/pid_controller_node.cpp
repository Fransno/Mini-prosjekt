#include "qube_controller/pid_controller_node.hpp"

PIDControllerNode::PIDControllerNode()
    : Node("pid_controller_node") {
    // Declare and get PID parameters
    this->declare_parameter<double>("p", 0.5);
    this->declare_parameter<double>("i", 0.1);
    this->declare_parameter<double>("d", 0.01);
    this->declare_parameter<double>("reference", 0.0);
    
    double p = this->get_parameter("p").as_double();
    double i = this->get_parameter("i").as_double();
    double d = this->get_parameter("d").as_double();
    double reference = this->get_parameter("reference").as_double();

    // Initialize PID controller
    pid_controller_ = std::make_unique<PIDController>(p, i, d, reference);

    // Create publisher for velocity commands
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", 10);

    // Create subscriber for joint states
    joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&PIDControllerNode::joint_states_listener, this, std::placeholders::_1));
}

void PIDControllerNode::joint_states_listener(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.empty() || msg->velocity.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty joint states message");
        return;
    }

    // Hent både posisjon og hastighet
    double position = msg->position[0];
    double velocity = msg->velocity[0];
    
    // Beregn hastighetskommando med både posisjon og hastighet
    double velocity_command = pid_controller_->update(position, velocity);

    auto velocity_msg = std_msgs::msg::Float64MultiArray();
    
    // Sett opp layout (valgfritt, men anbefalt)
    velocity_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    velocity_msg.layout.dim[0].label = "velocity";
    velocity_msg.layout.dim[0].size = 1;
    velocity_msg.layout.dim[0].stride = 1;
    
    // Legg til data
    velocity_msg.data.push_back(velocity_command);

    // Publiser hastighetskommandoen
    velocity_publisher_->publish(velocity_msg);

    // Logg for debugging (valgfritt)
    RCLCPP_DEBUG(this->get_logger(), 
                "Position: %.3f, Velocity: %.3f, Command: %.3f",
                position, velocity, velocity_command);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}