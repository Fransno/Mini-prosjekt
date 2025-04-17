#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    // Constructor: Initializes the PID controller with specific parameters
    // p, i, d: Proportional, Integral, and Derivative gains
    // reference: The desired target value (e.g., position setpoint)
    PIDController(double p, double i, double d, double reference);

    // Update function: Takes in the current measured value and velocity
    double update(double measured_value, double measured_velocity);

    // Getters for the PID parameters 
    double getP() const { return p_; } 
    double getI() const { return i_; } 
    double getD() const { return d_; }

    // Setters for the PID parameters 
    void setP(double p) { p_ = p; } 
    void setI(double i) { i_ = i; } 
    void setD(double d) { d_ = d; } 

private:
    // Private member variables to store the PID parameters and state
    double p_;   
    double i_;  
    double d_;  
    double reference_; 
    double output_;  
    double integral_; 
    double previous_error_;
};

#endif // PID_CONTROLLER_HPP

