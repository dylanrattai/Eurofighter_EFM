#pragma once

class PID {
public:
    PID(double kp, double ki, double kd, double minOut, double maxOut)
        : Kp(kp), Ki(ki), Kd(kd), minOutput(minOut), maxOutput(maxOut),
        integral(0.0), prevError(0.0), firstRun(true) {
    }

    double compute(double setpoint, double measured, double dt) {
        double error = setpoint - measured;

        // --- Proportional term ---
        double P = Kp * error;

        // --- Integral term ---
        integral += error * dt;

        // Anti-windup clamp
        if (integral > maxOutput) integral = maxOutput;
        if (integral < minOutput) integral = minOutput;

        double I = Ki * integral;

        // --- Derivative term ---
        double derivative = 0.0;
        if (!firstRun)
            derivative = (error - prevError) / dt;

        double D = Kd * derivative;

        prevError = error;
        firstRun = false;

        // Total output
        double output = P + I + D;

        // Clamp final output
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;

        return output;
    }

private:
    double Kp, Ki, Kd;
    double minOutput, maxOutput;
    double integral;
    double prevError;
    bool firstRun;
};
