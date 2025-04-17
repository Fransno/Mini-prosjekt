#include "qube_controller/pid_controller_node.hpp"

// Constructor for the PIDControllerNode
// Initializes the PID controller with parameters, creates a publisher and a subscriber
PIDControllerNode::PIDControllerNode()
    : Node("pid_controller_node") {
    // Declare parameters for PID control (proportional, integral, derivative gains, and reference)
    // Default values are set in the declare_parameter calls
    this->declare_parameter<double>("p", 0.5); 
    this->declare_parameter<double>("i", 0.1); 
    this->declare_parameter<double>("d", 0.01); 
    this->declare_parameter<double>("reference", 0.0); 
    
    // Retrieve the PID parameters set by the user or the default values
    double p = this->get_parameter("p").as_double();
    double i = this->get_parameter("i").as_double();
    double d = this->get_parameter("d").as_double();
    double reference = this->get_parameter("reference").as_double();

    // Initialize the PID controller with the provided PID parameters
    pid_controller_ = std::make_unique<PIDController>(p, i, d, reference);

    // Create a publisher to send velocity commands to the velocity controller
    // This will publish messages to the "/velocity_controller/commands" topic
    velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", 10); // Queue size is set to 10

    // Create a subscriber to listen for joint states messages from the robot
    // This listens to the "/joint_states" topic and calls joint_states_listener callback when a message is received
    joint_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, // Queue size is set to 10
        std::bind(&PIDControllerNode::joint_states_listener, this, std::placeholders::_1)); // Bind the callback function
}

// Callback function for joint states
// This function is called when a new joint states message is received
void PIDControllerNode::joint_states_listener(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Check if the position or velocity data is empty, which indicates an invalid message
    if (msg->position.empty() || msg->velocity.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty joint states message");
        return;
    }

    // Extract the first position and velocity values from the joint state message
    double position = msg->position[0];
    double velocity = msg->velocity[0];
    
    // Use the PID controller to compute the velocity command based on the position and velocity
    double velocity_command = pid_controller_->update(position, velocity);

    // Create a message to send the velocity command to the velocity controller
    auto velocity_msg = std_msgs::msg::Float64MultiArray();
    
    // Optional: define layout for the velocity message for clarity/debugging
    velocity_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    velocity_msg.layout.dim[0].label = "velocity"; // Label for the dimension
    velocity_msg.layout.dim[0].size = 1; // Size of the dimension
    velocity_msg.layout.dim[0].stride = 1; // Stride of the dimension
    
    // Set the computed velocity command as the data of the message
    velocity_msg.data.push_back(velocity_command);

    // Publish the velocity command to the "/velocity_controller/commands" topic
    velocity_publisher_->publish(velocity_msg);

    // Optional: Debug log for displaying the received position, velocity, and the computed velocity command
    RCLCPP_DEBUG(this->get_logger(), 
                "Position: %.3f, Velocity: %.3f, Command: %.3f",
                position, velocity, velocity_command);
}

// Main function for ROS2 node execution
// Initializes the ROS2 context, spins the node, and shuts down the node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // Initialize ROS2
    auto node = std::make_shared<PIDControllerNode>(); // Create the node instance
    rclcpp::spin(node); // Keep the node running and processing incoming messages
    rclcpp::shutdown(); // Shutdown the ROS2 context once the node is done
    return 0;
}


