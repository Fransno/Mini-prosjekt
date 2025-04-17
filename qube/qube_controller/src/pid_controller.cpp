#include "qube_controller/pid_controller.hpp"

// Constructor for the PIDController class
PIDController::PIDController(double p, double i, double d, double reference):
    p_(p), i_(i), d_(d), reference_(reference),
    output_(0.0), integral_(0.0), previous_error_(0.0) {}

// Takes the current measured position and velocity
// Returns the control output (velocity command)
double PIDController::update(double measured_value, double measured_velocity) {
    const double dt = 0.01;
    double error = reference_ - measured_value;
    integral_ += error * dt;
    double derivative = -measured_velocity;
    output_ = p_ * error + i_ * integral_ + d_ * derivative;
    previous_error_ = error;
    
    return output_;
}
