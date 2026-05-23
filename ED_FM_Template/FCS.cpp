#include "Maths.h"
#include "FCS.h"

namespace {
    constexpr Flight_Control_System::FcsLimits subsonicLimits = {
        .maxG = 0.0,
        .maxAoa = 0.0,
        .maxRollRate = 0.0,
        .maxYawRate = 0.0,
        .maxPitchRate = 0.0
    };
    constexpr Flight_Control_System::FcsLimits supersonicLimits = {
        .maxG = 0.0,
        .maxAoa = 0.0,
        .maxRollRate = 0.0,
        .maxYawRate = 0.0,
        .maxPitchRate = 0.0
    };
    constexpr Flight_Control_System::FcsLimits landingLimits = {
        .maxG = 0.0,
        .maxAoa = 0.0,
        .maxRollRate = 0.0,
        .maxYawRate = 0.0,
        .maxPitchRate = 0.0
    };
    constexpr Flight_Control_System::FcsLimits refuelingLimits = {
        .maxG = 0.0,
        .maxAoa = 0.0,
        .maxRollRate = 0.0,
        .maxYawRate = 0.0,
        .maxPitchRate = 0.0
    };

    constexpr double maxPilotInputRate = 5; // units per second, TODO: tune this value based on feel and testing
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
    filteredCommandedPitch = 0.0;
    commandedPitch         = 0.0;
    nosewheelAngle         = 0.0;
    newCanardAnims         = 0.0;
    canardPosition         = 0.0;
    currentAoa             = 0.0;

    //                         P  I  D
    pitchController.initialize(1, 0, 0, -1.0, 1.0);
    //                        P  I  D
    rollController.initialize(3, 0, 0, -1.0, 1.0);
}

/**
 * @brief Cold start initialization. Runs a zeroInit.
 */
void Flight_Control_System::coldInit()
{
    zeroInit();
}

/**
 * @brief Hot start initialization. Runs a zeroInit.
 */
void Flight_Control_System::hotInit()
{
    zeroInit();
}

/**
 * @brief Airborne start initialization. Runs a zeroInit.
 */
void Flight_Control_System::airborneInit()
{
    zeroInit();
}

/**
 * @brief Filter pilot input with global rate limiters, does not rate limit throttle. 
 * 
 * @attention this function updates the prevFilteredInput to be replaced by the calculated filtered input!!!
 * 
 * @param rawInput Struct containing raw pilot input values for the current frame.
 * 
 * @return PilotInput struct containing filtered pilot input values for the current frame.
*/
Flight_Control_System::PilotInput Flight_Control_System::filterPilotInput(const PilotInput& rawInput, PilotInput& prevFilteredInput)
{
    PilotInput filteredInput;
    double maxDelta = maxPilotInputRate * m_dt;

    // Lambda function to apply slew rate limiting to a single axis
    auto slew = [maxDelta](double commandedInput, double prevFiltered) {
        double delta = commandedInput - prevFiltered;
        if(delta > maxDelta)
            return prevFiltered + maxDelta;
        else if(delta < -maxDelta)
            return prevFiltered - maxDelta;
        else
            return commandedInput;
    };

    // Apply slew rate limiting to each axis
    filteredInput.pitch = slew(rawInput.pitch, prevFilteredInput.pitch);
    filteredInput.roll = slew(rawInput.roll, prevFilteredInput.roll);
    filteredInput.yaw = slew(rawInput.yaw, prevFilteredInput.yaw);

    prevFilteredInput = filteredInput; // Update previous filtered input for next frame

    return filteredInput;
}

/**
 * @brief Selects the active FCS mode based off aircraft state.
 *
 * Priority: AAR law -> Landing law -> Supersonic law -> Subsonic law.
 * TODO: blending in trans sonic regime, and refueling door partially open states.
 * 
 * @param state Current aircraft state struct containing all relevant parameters for mode selection.
 * 
 * @return FcsLaw enum value corresponding to the active FCS mode.
 */
Flight_Control_System::FcsLaw Flight_Control_System::selectFcsLaw(const AircraftState& state)
{
    if (state.refuelingDoorPose > 0.1)
    {
        return FcsLaw::REFUELING_LAW;
    }
    else if (state.nosewheelAngle > 0.1)
    {
        return FcsLaw::LANDING_LAW;
    }
    else if (state.mach >= 1.0)
    {
        return FcsLaw::SUPERSONIC_LAW;
    }
    else
    {
        return FcsLaw::SUBSONIC_LAW;
    }
}

/**
 * @brief Updates axis limiters based on active FCS mode.
 * 
 * @param law Active FCS mode.
 * 
 * @return FcsLimits struct containing axis limit values for the active mode.
*/
Flight_Control_System::FcsLimits Flight_Control_System::updateLimitsForLaw(const FcsLaw& law)
{
    switch (law)
    {
        case FcsLaw::SUBSONIC_LAW:
            return subsonicLimits;
        case FcsLaw::SUPERSONIC_LAW:
            return supersonicLimits;
        case FcsLaw::LANDING_LAW:
            return landingLimits;
        case FcsLaw::REFUELING_LAW:
            return refuelingLimits;
        default:
            return subsonicLimits;
    }
}

// Main per-frame FCS update. Order matters: mode selection -> axis limiters -> actuator helpers.
void Flight_Control_System::update(double dt)
{
    m_dt = dt;

    // Update current aircraft state struct
    currentAircraftState = {
        .g = m_state.getNY(),
        .aoa = m_state.m_aoa,
        .mach = m_state.m_mach,
        .pitchRate = m_state.m_omega.z,
        .rollRate = m_state.m_omega.x,
        .yawRate = m_state.m_omega.y,
        .nosewheelAngle = m_airframe.getGearNPosition(),
        .refuelingDoorPose = m_airframe.getRefuelingDoor()
    };

    // Update raw pilot input struct
    pilotInputRaw = {
        .pitch = m_input.getPitch(),
        .roll = m_input.getRoll(),
        .yaw = m_input.getYaw(),
        .throttle1 = m_input.getThrottle(),
        .throttle2 = m_input.getThrottle2()
    };

    pilotInputFiltered = filterPilotInput(pilotInputRaw, pilotInputPrev);

    // Decide active FCS mode based on current aircraft state; then update limits.
    currentLaw = selectFcsLaw(currentAircraftState);
    currentLimits = updateLimitsForLaw(currentLaw);
}