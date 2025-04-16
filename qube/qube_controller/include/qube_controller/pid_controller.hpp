#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

// Simple PID controller class for controlling velocity based on joint state feedback
class PIDController {
public:

    // Constructor, reference: desired setpoint (position)
    PIDController(double p, double i, double d, double reference);

    // Update method
    double update(double measured_value, double measured_velocity);

private:
    // PID gains
    double p_;
    double i_;
    double d_;

    // Desired reference value (setpoint)
    double reference_;

    // Last computed control output
    double output_;

    // Integral term accumulator
    double integral_;

    // Previous error (not used in current implementation, but stored for future use)
    double previous_error_;
};

#endif // PID_CONTROLLER_HPP
