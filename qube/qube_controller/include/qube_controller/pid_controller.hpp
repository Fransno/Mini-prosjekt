#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(double p, double i, double d, double reference);
    double update(double measured_value, double measured_velocity);

private:
    double p_;
    double i_;
    double d_;
    double reference_;
    double output_;
    double integral_;
    double previous_error_;
};

#endif // PID_CONTROLLER_HPP
