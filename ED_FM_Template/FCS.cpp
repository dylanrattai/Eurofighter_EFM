#include "Maths.h" // For std::clamp
#include "FCS.h"

namespace
{
	constexpr double kMinSafeCurrentG { 1e-3 };
	constexpr double kPitchGLimiterBias { 5.0 };
	constexpr double kPitchCmdToTargetG { 4.0 };
	constexpr double kPitchAoaLimiterUpperBoundRad { 100.0 };

	constexpr double kRefuelingDoorOpenThreshold { 0.3 };
	constexpr double kNosewheelLandingDetectThreshold { 0.1 };
	constexpr double kSupersonicMachThreshold { 0.98 };

	constexpr double kYawDampingAoaStartDeg { 10.0 };
	constexpr double kYawAuthorityLanding { 0.05 };
	constexpr double kYawAuthorityRefueling { 0.25 };
	constexpr double kYawAuthorityNominal { 0.5 };

	constexpr double kRollAuthorityLanding { 0.5 };
	constexpr double kRollAuthoritySupersonic { 0.25 };
	constexpr double kRollAuthorityRefueling { 0.5 };

	constexpr double kCanardBlendAoaThresholdDeg { 5.0 };
	constexpr double kCanardTransitionTimeSeconds { 10.0 };
}

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

/**
 * @brief Initialize all internal values with a "zero" state.
 */
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

/**
 * @brief Cold start initialization. Runs a zeroInit
 */
void Flight_Control_System::coldInit()
{
	zeroInit();
}

/**
 * @brief Hot start initialization. Runs a zeroInit
 */
void Flight_Control_System::hotInit()
{
    zeroInit();
}

/**
 * @brief Airborne start initialization. Runs a zeroInit
 */
void Flight_Control_System::airborneInit()
{
	zeroInit();
}

/**
 * @brief TODO: Document this function.
 */
// Applies stick-command limiting and computes the pitch PID setpoint.
// Limits are based on current G and AoA envelope for the active FCS mode.
void Flight_Control_System::limit_pitch()
{
	// Work on a filtered copy; raw pilot input is preserved in pitchcmd.

	pitch_cmd_filtered = pitchcmd;

	// Hard G limiter. Prevent divide-by-zero and unstable gains around near-zero G.

	double safe_current_g { current_g };
	if (safe_current_g < kMinSafeCurrentG)
	{
		safe_current_g = kMinSafeCurrentG;
	}

	// Reduce commanded pitch as configured G and AoA limits are approached.
	double scale_factor { (max_g + kPitchGLimiterBias) / safe_current_g };
	if (scale_factor < 1)
	{
		pitch_cmd_filtered *= limit(scale_factor, 0.0, 1.0);
	}
	scale_factor = max_AoA / limit(current_aoa, 0.0, kPitchAoaLimiterUpperBoundRad);
	if (scale_factor < 1)
	{
		pitch_cmd_filtered *= limit(scale_factor, 0.0, 1.0);
	}
	pitch_cmd_filtered = limit(pitch_cmd_filtered, -1.0, 1.0);

	// Convert normalized pitch command to target normal-acceleration request.
	double target_g { pitch_cmd_filtered * kPitchCmdToTargetG };
	pitchController.update(target_g, current_g, m_dt);
	//pitch_cmd_filtered = pitchController.getOutputPID() / 4; Temporarly comment
}

/**
 * @brief Takes in raw yaw input and applies limiting/damping based on active FCS mode and current AoA.
 */
void Flight_Control_System::limit_yaw()
{
	yaw_cmd_filtered = yawcmd;
	const double yaw_damping_aoa_start { kYawDampingAoaStartDeg * DEG_TO_RAD };

	if (landing_FCS_mode == 1.0)
	{
		yaw_cmd_filtered *= kYawAuthorityLanding;
	}
	else if (refueling_FCS_mode == 1.0)
	{
		yaw_cmd_filtered *= kYawAuthorityRefueling;
	}
	else
	{
		yaw_cmd_filtered *= kYawAuthorityNominal;
	}

	// Additional damping at high AoA to reduce departure risk.
	if (current_aoa > yaw_damping_aoa_start)
	{
		double scale_factor { yaw_damping_aoa_start / current_aoa };
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

/**
 * @brief Takes in raw roll input and applies damping based on active FCS mode. Then runs the roll PID controller to compute the final roll command output.
 */
void Flight_Control_System::limit_roll()
{
	roll_cmd_filtered = rollcmd;
	if (landing_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= kRollAuthorityLanding;
	}
	else if (subsonic_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= 1;
	}
	else if (supersonic_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= kRollAuthoritySupersonic;
	}
	else if (refueling_FCS_mode == 1.0)
	{
		roll_cmd_filtered *= kRollAuthorityRefueling;
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

/** 
 * @brief Selects the active FCS mode based off aircraft state.
 * 
 * Priority: AAR law -> Landing law -> Supersonic law -> Subsonic law.
*/
void Flight_Control_System::limiter_mode()
{
	// Mode selection priority: refueling door -> landing gear state -> Mach regime.
	if (m_airframe.getRefuelingDoor() < kRefuelingDoorOpenThreshold)
	{
		if (nosewheel_angle > kNosewheelLandingDetectThreshold)
		{
			landing_FCS_mode = 1.0; //used for canard animations based on FCS mode
			supersonic_FCS_mode = 0.0;
			subsonic_FCS_mode = 0.0;
			refueling_FCS_mode = 0.0;
			landing_limit();
		}
		else if (m_state.m_mach > kSupersonicMachThreshold)
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

/**
 * @brief Sets the limits for subsonic flight law
 */
void Flight_Control_System::subsonic_limit()
{
	// OLD Subsonic maneuvering envelope.
	limited_roll_rate = 200.0 * DEG_TO_RAD;


	//----- NEW FBW LIMITS  ------
	max_AoA = 30 * DEG_TO_RAD;
	max_g = 4.45; // 7.25 G
	max_neg_g = -1;

	max_current_pitch_rate = 15 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

/**
 * @brief Sets the limits for landing flight law
 */
void Flight_Control_System::landing_limit()
{
	// OLD Landing envelope prioritizes controllability and low-rate authority.
	limited_roll_rate = 80.0 * DEG_TO_RAD;
	

	//----- NEW FBW LIMITS  ------
	max_AoA = 15 * DEG_TO_RAD;
	max_g = 4; // 4 G
	max_neg_g = -0;

	max_current_pitch_rate = 15 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

/**
 * @brief Sets the limits for supersonic flight law
 */
void Flight_Control_System::supersonic_limit()
{
	// OLD Supersonic envelope limits pitch rate to control structural loads.
	limited_roll_rate = 200.0 * DEG_TO_RAD;
	

	//----- NEW FBW LIMITS  ------
	max_AoA = 15 * DEG_TO_RAD;
	max_g = 9.0 ; // 9 G
	max_neg_g = -1;

	max_current_pitch_rate = 7 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

/**
 * @brief Sets the limits for Air to Air Refueling flight law
 */
void Flight_Control_System::refueling_limit()
{
	// OLD Refueling envelope with reduced agility for station-keeping.
	limited_roll_rate = 80.0 * DEG_TO_RAD;

	//----- NEW FBW LIMITS  ------
	max_g = 2.0 ; //2 G
	max_neg_g = -0.0;
	max_AoA = 15 * DEG_TO_RAD;

	max_current_pitch_rate = 15 * DEG_TO_RAD;
	min_current_pitch_rate = -2 * DEG_TO_RAD;
}

/**
 * @brief TODO: Document this function.
 */
void Flight_Control_System::autoDriveCanardPosition()
{
	const double canard_aoa_threshold { kCanardBlendAoaThresholdDeg * DEG_TO_RAD };
	double transition_speed { m_dt / kCanardTransitionTimeSeconds };
	canard_position = new_canard_anims;

	// Blend canard animation in/out based on AoA threshold.
	if (canard_position < 1 && m_state.m_aoa > canard_aoa_threshold)
	{
		new_canard_anims = limit(new_canard_anims + transition_speed, 0.0, 1.0);
	}
	else if (canard_position >= 1 && m_state.m_aoa > canard_aoa_threshold)
	{
		new_canard_anims = 1;
	}
	else if (canard_position >= 1 && m_state.m_aoa < canard_aoa_threshold)
	{
		new_canard_anims = limit(new_canard_anims - transition_speed, 0.0, 1.0);
	}
	else if (canard_position <= 0 && m_state.m_aoa < canard_aoa_threshold)
	{
		new_canard_anims = 0;
	}
}

// Main per-frame FCS update. Order matters: mode selection -> axis limiters -> actuator helpers.

void Flight_Control_System::update(double dt)
{
	m_dt = dt;

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

}


