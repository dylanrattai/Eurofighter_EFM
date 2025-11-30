#include <iostream>
#include <algorithm>  // for std::clamp

class PID {
public:
    PID(double Kp, double Ki, double Kd,
        double outputMin = -1e9, double outputMax = 1e9)
        : Kp(Kp), Ki(Ki), Kd(Kd),
        outputMin(outputMin), outputMax(outputMax) {
    }

    double compute(double setpoint, double measurement, double dt) {

        // Calculate error
        double error = setpoint - measurement;

        // --- Proportional term ---
        double P = Kp * error;

        // --- Integral term with clamping ---
        integral += error * dt;
        integral = clamp(integral, -1000.0, 1000.0); // anti-windup
        double I = Ki * integral;

        // --- Derivative term ---
        double derivative = (error - previousError) / dt;
        double D = Kd * derivative;

        // --- Total output ---
        double output = P + I + D;
        output = clamp(output, outputMin, outputMax);

        previousError = error;
        return output;
    }

    void reset()
    {
        integral = 0.0;
        previousError = 0.0;
    }

private:
    double Kp, Ki, Kd;
    double integral = 0.0;
    double previousError = 0.0;

    double outputMin;
    double outputMax;
};
