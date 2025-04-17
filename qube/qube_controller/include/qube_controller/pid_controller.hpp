#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
public:
    PIDController(double p, double i, double d, double reference);
    double update(double measured_value, double measured_velocity);

    // Getters for the PID parameters (optional, in case you need them)
    double getP() const { return p_; }
    double getI() const { return i_; }
    double getD() const { return d_; }

    void setP(double p) { p_ = p; }
    void setI(double i) { i_ = i; }
    void setD(double d) { d_ = d; }

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
