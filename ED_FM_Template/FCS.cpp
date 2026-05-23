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

    pitchRate      = m_state.m_omega.z;
    rollRate       = m_state.m_omega.x;
    currentG       = m_state.getNY();
    nosewheelAngle = m_airframe.getGearNPosition();
    commandedPitch = m_input.getPitch();
    commandedRoll  = m_input.getRoll();
    commandedYaw   = m_input.getYaw();
    //commandedThrottle1 = m_input.getThrottle();
    //commandedThrottle2 = m_input.getThrottle2();
    currentAoa  = m_state.m_aoa;
    currentMach = m_state.m_mach;
    //wingStall = m_flight_model.getWingstall();

    // Update current aircraft state struct for use in mode selection and axis limiters.
    currentAircraftState = {
        .g = currentG,
        .aoa = currentAoa,
        .mach = currentMach,
        .pitchRate = pitchRate,
        .rollRate = rollRate,
        .yawRate = m_state.m_omega.y,
        .nosewheelAngle = nosewheelAngle,
        .refuelingDoorPose = m_airframe.getRefuelingDoor()
    };

    // Decide active FCS mode based on current aircraft state; then update limits.
    currentLaw = selectFcsLaw(currentAircraftState);
    currentLimits = updateLimitsForLaw(currentLaw);
}