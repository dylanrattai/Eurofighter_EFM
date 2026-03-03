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
	//                         P  I  D
	pitchController.initialize(1, 0, 0, -1.0, 1.0);
	//                        P  I  D
	rollController.initialize(3, 0, 0, -1.0, 1.0);

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

	pitch_cmd_filtered = pitchcmd;

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
	double target_g = pitch_cmd_filtered * 4;//(1 + pitch_cmd_filtered * 100 / 12.5);
	pitchController.update(target_g, current_g, m_dt);
	//pitch_cmd_filtered = pitchController.getOutputPID() / 4; Temporarly comment
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
	roll_cmd_filtered = roll_cmd_filtered * (200 * DEG_TO_RAD);
	//printf("Roll command: %f \n", rollcmd);
	//printf("Roll command to deg: %f \n", roll_cmd_filtered / DEG_TO_RAD);
	//printf("Roll rate deg: %f \n", roll_rate / DEG_TO_RAD);
	rollController.update(roll_cmd_filtered, roll_rate, m_dt);
	roll_cmd_filtered = -(rollController.getOutputPID() / (200 * DEG_TO_RAD));
	rollController.debug();
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


