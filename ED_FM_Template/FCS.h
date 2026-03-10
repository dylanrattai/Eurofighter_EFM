#pragma once
#include "Vec3.h"
#include "State.h"
#include "Input.h"
#include "Airframe.h"
#include "BaseComponent.h"
#include "Pid.h"

class Flight_Control_System
{
public:
	Flight_Control_System(State& state, Input& input, Airframe& airframe);
    virtual void zeroInit();
    virtual void coldInit();
    virtual void hotInit();
    virtual void airborneInit();
    void update(double dt);
	void limit_pitch();
    void limit_roll();
    void limit_yaw();
    void limiter_mode();
    void subsonic_limit();
    void supersonic_limit();
    void landing_limit();
    void refueling_limit();
    //void low_speed_recovery();
    void autoDriveCanardPosition();
    //double PID_controller_pitch(double target, bool is_neg);
    //void PID_controller_roll(double target, double meassurement, double kp, double ki, double kd);
    //void PID_controller_yaw(double target, double meassurement, double kp, double ki, double kd);
    //double Flight_Control_System::Canard_AOA();
    double const Flight_Control_System::getpitch() const
    {
        return limit(pitch_cmd_filtered,-1,1);
    }
    double const Flight_Control_System::getroll() const
    {
        return limit(roll_cmd_filtered, -1, 1);;
    }
    double const Flight_Control_System::getyaw() const
    {
        return limit(roll_cmd_filtered, -1, 1);;
    }
    double const Flight_Control_System::get_canard_anim() const
    {
        return new_canard_anims;
    }
    double const Flight_Control_System::getThrottle1() const
    {
        return limit(throttle_cmd_filtered_1, -1, 1);
    }
    double const Flight_Control_System::getThrottle2() const
    {
        return limit(throttle_cmd_filtered_2, -1, 1);
    }

	PID pitchController;
    PID rollController;

// for testing purposes, expose internal state of FCS modes and limits
protected:
    double getCurrentG() const { return current_g; }
    double getCurrentAoA() const { return current_aoa; }
    double getPitchCmdFiltered() const { return pitch_cmd_filtered; }
    double getRollCmdFiltered() const { return roll_cmd_filtered; }
    double getYawCmdFiltered() const { return yaw_cmd_filtered; }
    double getMaxAoA() const { return max_AoA; }
    double getMaxG() const { return max_g; }
    double getMaxNegG() const { return max_neg_g; }
    double getLimitedRollRate() const { return limited_roll_rate; }
    double getCanardPosition() const { return canard_position; }
    double getLandingFCSMode() const { return landing_FCS_mode; }
    double getSupersonicFCSMode() const { return supersonic_FCS_mode; }
    double getSubsonicFCSMode() const { return subsonic_FCS_mode; }
    double getRefuelingFCSMode() const { return refueling_FCS_mode; }

private:
    State& m_state;
    Input& m_input;
    Airframe& m_airframe;

    

    double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

    double throttlecmd_1 = 0.0;
    double throttle_cmd_filtered_1 = 0.0;
    double throttlecmd_2 = 0.0;
    double throttle_cmd_filtered_2 = 0.0;
    double airspeed = 0.0;

    //bool wing_stall = false;

	double previous_aoa = 0.0;
	double pitch_cmd_filtered = 0.0;
    double roll_cmd_filtered = 0.0;
    double new_canard_anims = 0.0;
    double canard_position = 0.0;
    double max_g = 0.0;
    double max_AoA = 0.0;
    double max_neg_g = 0.0;
    double current_g = 1.0;
    double pitch_rate = 0.0;

	double AOA_BUFFER_ZONE = 0.05; // 3° in radians
	double BLEND_RATE = 0.5; // Blend rate for soft limit zone

	double pitchcmd = 0.0; // Pilot's pitch command
	double current_aoa = 0.0; // Current angle of attack
	double mach = 0.0; // Current Mach number
	double nosewheel_angle = 0.0; // Nosewheel gear angle (Used to check if wheels are out)
	double pi = 3.14159265358979323846;
    double m_dt = 0.0;

    const double GROUND_AOA_MAX = 0.35;    // 20° in radians
    const double GROUND_AOA_MIN = -0.09;    // -5° in radians
    const double LANDING_AOA_LIMIT = 0.3142; // 18° in radians
    const double DEFAULT_AOA_LIMIT = 0.4189; // 24° in radians#

    //Pitch pid
    double pitch_meassurement_prior = 0.0;
    double pitch_derivative_prior = 0.0;
    double pitch_error_prior = 0.0;
    double pitch_error = 0.0;
    double pitch_PID_value_out = 0.0;
    double pitch_integral_prior = 0.0;
    double pitch_pid_result = 0.0;
    double pitch_target_prev = 0.0;

    //Roll pid
    double roll_meassurement_prior = 0.0;
    double roll_derivative_prior = 0.0;
    double roll_error_prior = 0.0;
    double roll_error = 0.0;
    double roll_PID_value_out = 0.0;
    double roll_integral_prior = 0.0;
    double roll_pid_result = 0.0;

    //Yaw pid
    double yaw_meassurement_prior = 0.0;
    double yaw_derivative_prior = 0.0;
    double yaw_error_prior = 0.0;
    double yaw_error = 0.0;
    double yaw_PID_value_out = 0.0;
    double yaw_integral_prior = 0.0;
    double yaw_pid_result = 0.0;

    //------------------------------------------
    double subsonic_FCS_mode = 0.0;
    double landing_FCS_mode = 0.0;
    double supersonic_FCS_mode = 0.0;
    double refueling_FCS_mode = 0.0;

    double rollcmd = 0.0;
    double roll_rate = 0.0;
    double limited_roll_rate = 0.0;

    double yawcmd = 0.0;
    double yaw_cmd_filtered = 0.0;




    //----------------------------------------------

    double max_current_pitch_rate = 0.0;
    double max_current_roll_rate = 0.0;
    double max_current_yaw_rate = 0.0; //These will be in RADIANS

    double min_current_pitch_rate = 0.0;
    double min_current_roll_rate = 0.0;
    double min_current_yaw_rate = 0.0;

    // Mach-AoA limit table (radians)
    

    static double lerp(double* x, double* f, unsigned sz, double t)
    {
        for (unsigned i = 0; i < sz; i++)
        {
            if (t <= x[i])
            {
                if (i > 0)
                {
                    return ((f[i] - f[i - 1]) / (x[i] - x[i - 1]) * t +
                        (x[i] * f[i - 1] - x[i - 1] * f[i]) / (x[i] - x[i - 1]));
                }
                return f[0];
            }
        }
        return f[sz - 1];
    }

    static double lerpRadians(float a, float b, float t) {
        t = clamp(t, 0.0, 1.0);  // C++17 clamp
        return a + t * (b - a);
    }

    static double limit(double val, double low, double high)
    {
        if (val < low) { return low; }
        else if (val > high) { return high; }
        else { return val; }
    }

    double stick_to_g(double stick)
    {
        stick = clamp(stick, -1.0, 1.0);

        double g_cmd;

        if (stick >= 0.0)
            g_cmd = 1.0 + stick * (max_g - 1.0);     // 1 -> +9
        else
            g_cmd = 1.0 + stick * (1.0 - max_neg_g); // 1 -> -2

        return g_cmd;
    }
    static double max(double x, double y) 
    {
        if (x > y) {
            return x;
        }
        else
        {
            return y;
        }
    }
    double g_error_to_actuator(double g_err)
    {
        double g_range = max(max_g - 1.0, 1.0 - max_neg_g);
        return clamp(g_err / g_range, -1.0, 1.0);
    }

};


