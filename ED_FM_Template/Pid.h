#include <iostream>
#include <algorithm>

class PID {
public:
    PID() = default;
    virtual ~PID() = default;

    void initialize(double Kp, double Ki, double Kd,
        double outputMin = -1e9, double outputMax = 1e9)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->outputMin = outputMin;
        this->outputMax = outputMax;
    }

    double update(double setpoint, double value, double dt, bool inverted_logic = false) {
        double error = inverted_logic ? setpoint - value : value - setpoint;
        
        if (dt <= 1e-6) return value_out; //Prevents zero errors

        total_time += dt;

        P = Kp * error;

        integral += error * dt;
        I = Ki * integral;

        double derivative = (error - prior_error) / dt;
        D = Kd * derivative;

        double value_out = P + I + D;
        value_out = clamp(value_out, outputMin, outputMax);

        prior_error = error;
        prior_proportional = P;
        prior_integral = I;
        prior_derivative = D;

        this->value_out = value_out;
        return value_out;
    }

    void reset()
    {
        integral = 0.0;
        total_time = 0.0;
        prior_error = 0.0;
        prior_proportional = 0.0;
        prior_integral = 0.0;
        prior_derivative = 0.0;
    }

    void debug()
    {
        printf("Error: %f | ", prior_error);
        printf("Proportional: %f | ", P);
        printf("Integral: % f | ", I);
        printf("Derivative: % f | ", D);
        printf("Value out: % f | ", value_out);
    }

    double getOutputPID() { return value_out; }

private:
    double Kp, Ki, Kd = 0.0;
    double P = 0.0;
    double I = 0.0;
    double D = 0.0;
    double tau = 0.0;
    double value_out = 0.0;
    double total_time = 0.0;
    double integral = 0.0;
    double prior_error = 0.0;
    double prior_proportional = 0.0;
    double prior_integral = 0.0;
    double prior_derivative = 0.0;
    double Pi = 3.1415926536;
    double outputMin = 0.0;
    double outputMax = 0.0;
    double LPF = 0.0;
    double HPF = 0.0;
};