#include "Maths.h" // For std::clamp
#include "FCS.h"

Flight_Control_System::Flight_Control_System
(
	State& state, 
	Input& input, 
	Airframe& airframe
): 
	m_state(state),
	m_input(input), 
	m_airframe(airframe)
{

}

void Flight_Control_System::zeroInit()
{
	pitch_cmd_filtered = 0.0;
	pitchcmd = 0.0; // Initialize pitch command
	nosewheel_angle = 0.0;
	new_canard_anims = 0.0;
	canard_position = 0.0;
	current_aoa = 0.0;


	pitch_error_prior = 0.0;
	pitch_error = 0.0;
	pitch_derivative_prior = 0.0;
	pitch_pid_result = 0.0;
	pitch_integral_prior = 0; // or small value
	pitch_error_prior = pitch_error;
	pitch_meassurement_prior = pitch_rate;

}
void Flight_Control_System::coldInit()
{
	zeroInit();
}
void Flight_Control_System::hotInit()
{
    zeroInit();
}
void Flight_Control_System::airborneInit()
{
	zeroInit();
}

//Revised FBW System 
void Flight_Control_System::limit_pitch()
{
	//Assign pitchcmd to filtered to make life easier

	pitch_cmd_filtered = -pitchcmd;

	//Hard limiter for G (Safety)

	double scale_factor = (max_g + 5) / current_g;
	if (scale_factor < 1)
	{
		pitch_cmd_filtered *= limit(scale_factor, 0.0, 1.0);
	}
	scale_factor = max_AoA / limit(current_aoa, 0.0, 100.0);
	if (scale_factor < 1)
	{
		pitch_cmd_filtered *= limit(scale_factor, 0.0, 1.0);
	}
	pitch_cmd_filtered = limit(pitch_cmd_filtered, -1.0, 1.0);

	// Assign pitch to a pitch range
	//pitch_cmd_filtered *= max_g; OLD
	bool is_neg = false;

	//Run the pid
	pitch_cmd_filtered = PID_controller_pitch(pitch_cmd_filtered, is_neg);
}

void Flight_Control_System::limit_yaw()
{
	yaw_cmd_filtered = yawcmd;
	if (landing_FCS_mode == 1.0)
	{
		yaw_cmd_filtered *= 0.05;
	}
	else if (refueling_FCS_mode == 1.0)
	{
		yaw_cmd_filtered *= 0.25;
	}
	else
	{
		yaw_cmd_filtered *= 0.5;
	}

	if (current_aoa > (10 * DEG_TO_RAD))
	{
		double scale_factor = (10 * DEG_TO_RAD) / current_aoa;
		yaw_cmd_filtered *= scale_factor;
	}
}

//void Flight_Control_System::low_speed_recovery()
//{
//	if (airspeed <= 0.209977)
//	{
//		throttle_cmd_filtered_1 = 1.0;
//		throttle_cmd_filtered_2 = 1.0;
//	}
//	else
//	{
//		throttle_cmd_filtered_1 = throttlecmd_1;
//		throttle_cmd_filtered_2 = throttlecmd_2;
//	}
//}

void Flight_Control_System::limit_roll()
{
	roll_cmd_filtered = rollcmd;
	//REALLY TEMPORARY
	if (landing_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 0.5;
	}
	else if (subsonic_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 1;
	}
	else if (supersonic_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 0.25;
	}
	else if (refueling_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 0.5;
	}
	roll_cmd_filtered = limit(roll_cmd_filtered, -1.0, 1.0);
}

void Flight_Control_System::limiter_mode()
{
	if (m_airframe.getRefuelingDoor() < 0.3)
	{
		if (nosewheel_angle > 0.1)
		{
			landing_FCS_mode = 1.0; //used for canard animations based on FCS mode
			supersonic_FCS_mode = 0.0;
			subsonic_FCS_mode = 0.0;
			refueling_FCS_mode = 0.0;
			landing_limit();
		}
		else if (m_state.m_mach > 0.98)
		{
			landing_FCS_mode = 0.0;
			supersonic_FCS_mode = 1.0;
			subsonic_FCS_mode = 0.0;
			refueling_FCS_mode = 0.0;
			supersonic_limit();
		}
		else
		{
			landing_FCS_mode = 0.0; 
			supersonic_FCS_mode = 0.0;
			subsonic_FCS_mode = 1.0;
			refueling_FCS_mode = 0.0;
			subsonic_limit();
		}
	}
	else
	{
		landing_FCS_mode = 0.0;
		supersonic_FCS_mode = 0.0;
		subsonic_FCS_mode = 0.0;
		refueling_FCS_mode = 1.0;
		refueling_limit();
	}
}

void Flight_Control_System::subsonic_limit()
{
	// OLD
	//Set max limits for this mode
	limited_roll_rate = 200.0 * DEG_TO_RAD;


	//----- NEW FBW LIMITS  ------
	max_AoA = 30 * DEG_TO_RAD;
	max_g = 4.45; // 7.25 G
	max_neg_g = -1;

	max_current_pitch_rate = 15 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

void Flight_Control_System::landing_limit()
{
	// OLD
	//Set max limits for this mode
	limited_roll_rate = 80.0 * DEG_TO_RAD;
	

	//----- NEW FBW LIMITS  ------
	max_AoA = 15 * DEG_TO_RAD;
	max_g = 4; // 4 G
	max_neg_g = -0;

	max_current_pitch_rate = 15 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

void Flight_Control_System::supersonic_limit()
{
	// OLD
	//Set max limits for this mode
	limited_roll_rate = 200.0 * DEG_TO_RAD;
	

	//----- NEW FBW LIMITS  ------
	max_AoA = 15 * DEG_TO_RAD;
	max_g = 9.0 ; // 9 G
	max_neg_g = -1;

	max_current_pitch_rate = 7 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

void Flight_Control_System::refueling_limit()
{
	// OLD
	//Set max limits for this mode
	limited_roll_rate = 80.0 * DEG_TO_RAD;

	//----- NEW FBW LIMITS  ------
	max_g = 2.0 ; //2 G
	max_neg_g = -0.0;
	max_AoA = 15 * DEG_TO_RAD;

	max_current_pitch_rate = 15 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

void Flight_Control_System::autoDriveCanardPosition()
{
	double transition_speed = m_dt / 10;
	canard_position = new_canard_anims;
	// Move the canards to assist between 8 and 5 degrees
	if (canard_position < 1 && m_state.m_aoa >(5 * DEG_TO_RAD))
	{
		new_canard_anims += transition_speed;
	}
	else if (canard_position >= 1 && m_state.m_aoa > (5 * DEG_TO_RAD))
	{
		new_canard_anims = 1;
	}
	else if (canard_position >= 1 && m_state.m_aoa < (5 * DEG_TO_RAD))
	{
		new_canard_anims = transition_speed - new_canard_anims;
	}
	else if (canard_position <= 0 && m_state.m_aoa < (5 * DEG_TO_RAD))
	{
		new_canard_anims = 0;
	}
}

// The PID's --------------------------------------

//Pitch PID
double Flight_Control_System::PID_controller_pitch(double target, bool is_neg)
{
	// Validate delta time
	if (m_dt <= 1e-6) {
		return pitch_pid_result;
	}
	// Convert stick position to target G
	double target_g = (1 + target * 100 / 12.5);

	double measurement = current_g;
	printf("G mesurement from FBW: %f \n ", measurement);

	// PID gains - TUNED FROM PYTHON SIMULATION
	double kp = 0.28;
	double ki = 0.25;
	double kd = 0.15;
	double tau = 0.02;

	// Calculate error
	pitch_error = target_g - measurement;

	// Proportional term
	double proportional = kp * pitch_error;

	// Integral term with anti-windup
	double integral = pitch_integral_prior + ki * pitch_error * m_dt;
	//printf("Pitch integral from FBW: %f \n ", integral);
	printf("Pitch error from FBW: %f \n ", pitch_error);
	integral = clamp(integral, -0.1, 0.5);

	// Derivative term (on measurement to avoid derivative kick)
	double raw_derivative = -(measurement - pitch_meassurement_prior) / m_dt;
	// Filter the raw derivative first, THEN apply gain
	double derivative = pitch_derivative_prior +
		(m_dt / (tau + m_dt)) * (raw_derivative - pitch_derivative_prior);
	derivative *= kd;
	printf("Pitch derivative from FBW: %f \n ", derivative);
	derivative = clamp(derivative, -0.1, 0.1);
	// Combine PID terms
	double value_out = proportional + integral + derivative;

	// Update state for next iteration
	pitch_integral_prior = integral;
	pitch_error_prior = pitch_error;
	pitch_meassurement_prior = measurement;
	pitch_derivative_prior = derivative;

	// Output is already in -1 to 1 range from the PID
	pitch_pid_result = clamp(value_out, -1.0, 1.0);

	return pitch_pid_result;
}


//Roll PID

//void Flight_Control_System::PID_controller_roll(double target, double meassurement, double kp, double ki, double kd)
//{
//	//--------------PID------------------------
//	double tau = 0;
//
//	roll_error = target - meassurement;
//
//	double proportional = kp * roll_error;
//
//	double integral = roll_integral_prior + 0.5 * ki * (roll_error + roll_error_prior);
//
//	// Add anti-windup clamp
//	double integral_max = 100.0;  // example, tune based on actuator limits
//	double integral_min = -100.0;
//	integral = clamp(integral, integral_min, integral_max);
//
//
//	double alpha = (2.0 * tau - m_dt) / (2.0 * tau + m_dt);
//	double beta = (2.0 * kd) / (2.0 * tau + m_dt);
//
//	double derivative = alpha * roll_derivative_prior - beta * (meassurement - roll_meassurement_prior);
//
//	double value_out = proportional + integral + derivative;
//
//	roll_meassurement_prior = meassurement;
//	roll_error_prior = pitch_error;
//	roll_integral_prior = integral;
//	roll_derivative_prior = derivative;
//	//----------------END OF PID--------------------
//	roll_pid_result = value_out;
//}

//Yaw PID

//void Flight_Control_System::PID_controller_yaw(double target, double meassurement, double kp, double ki, double kd)
//{
//	//--------------PID------------------------
//	double tau = 0;
//
//	yaw_error = target - meassurement;
//
//	double proportional = kp * yaw_error;
//
//	double integral = yaw_integral_prior + 0.5 * ki * (yaw_error + yaw_error_prior);
//
//	// Add anti-windup clamp
//	double integral_max = 100.0;  // example, tune based on actuator limits
//	double integral_min = -100.0;
//	integral = clamp(integral, integral_min, integral_max);
//
//
//	double alpha = (2.0 * tau - m_dt) / (2.0 * tau + m_dt);
//	double beta = (2.0 * kd) / (2.0 * tau + m_dt);
//
//	double derivative = alpha * yaw_derivative_prior - beta * (meassurement - yaw_meassurement_prior);
//
//	double value_out = proportional + integral + derivative;
//
//	yaw_meassurement_prior = meassurement;
//	yaw_error_prior = pitch_error;
//	yaw_integral_prior = integral;
//	yaw_derivative_prior = derivative;
//	//----------------END OF PID--------------------
//	roll_pid_result = value_out;
//}

//----------------------------------------------------

void Flight_Control_System::update(double dt)
{
	pitch_rate = m_state.m_omega.z;
	roll_rate = m_state.m_omega.x; //convert to degrees for ease of use
	current_g = m_state.getNY();
	nosewheel_angle = m_airframe.getGearNPosition();
	pitchcmd = m_input.getPitch();
	rollcmd = m_input.getRoll();
	yawcmd = m_input.getYaw();
	//throttlecmd_1 = m_input.getThrottle();
	//throttlecmd_2 = m_input.getThrottle2();
	current_aoa = m_state.m_aoa;
	airspeed = m_state.m_mach; 
	//wing_stall = m_flight_model.getWingstall();
	limiter_mode();
	limit_roll();
	limit_yaw();
	limit_pitch();
	//low_speed_recovery();
	autoDriveCanardPosition();
    m_dt = dt;
}


