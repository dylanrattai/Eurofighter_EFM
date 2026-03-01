#include <iostream>
#include <algorithm>  // for std::clamp

class PID {
public:
    PID(double Kp, double Ki, double Kd,
        double outputMin = -1e9, double outputMax = 1e9)
        : Kp(Kp), Ki(Ki), Kd(Kd),
        outputMin(outputMin), outputMax(outputMax) {
    }

    double update(double setpoint, double value, double dt, bool inverted_logic = false) {

        // Calculate error
        double error = inverted_logic ? setpoint - value : value - setpoint;

        total_time += dt;
        // --- Proportional term ---
        P = Kp * error;

        // --- Integral term with clamping ---
        double integral += error * dt;
        I = Ki * integral;

        // --- Derivative term ---
        double derivative = (error - prior_error) / dt;
        D = Kd * derivative;

        // --- Total output ---
        double value_out = P + I + D;
        value_out = clamp(value_out, outputMin, outputMax);

        prior_error         = error;
        prior_proportional  = P;
        prior_integral      = I;
        prior_derivative    = D;
        
        double prior_value_out = value_out;

        return value_out;
    }

    void reset()
    {
        total_time = 0.0;
        prior_error = 0.0;
        prior_proportional = 0.0;
        prior_integral = 0.0;
        prior_derivative = 0.0;
    }

    double getOutputPID() { return value_out; }

private:
    double Kp, Ki, Kd   = 0.0;
    double P = 0.0;
    double I            = 0.0;
    double D            = 0.0;
    double tau          = 0.0; //Time constant
    double value_out    = 0.0;

    double total_time = 0.0;

    double prior_error          = 0.0;
    double prior_proportional   = 0.0;
    double prior_integral       = 0.0;
    double prior_derivative     = 0.0;

    double Pi                   = 3.1415926536;

    double outputMin    = 0.0; //Amplitude filters to limit max controller output
    double outputMax    = 0.0; //Amplitude filters to limit max controller output

    double LPF          = 0.0; //Frequency filters to damp the oscilations of controller
    double HPF          = 0.0; //Frequency filters to damp the oscilations of controller
};
