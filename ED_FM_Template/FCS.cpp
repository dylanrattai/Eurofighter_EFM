#include "Maths.h"
#include "FCS.h"
#include <algorithm>

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
    constexpr Flight_Control_System::PidValues betaZeroPValues = {
        .kP = 1.0,
        .kI = 0.0,
        .kD = 0.0
    };
    constexpr Flight_Control_System::PidValues rollRatePValues = {
        .kP = 1.0,
        .kI = 0.0,
        .kD = 0.0
    };
    constexpr Flight_Control_System::PidValues pitchRatePValues = {
        .kP = 1.0,
        .kI = 0.0,
        .kD = 0.0
    };
    constexpr Flight_Control_System::PidValues pitchLimiterPValues = {
        .kP = 1.0,
        .kI = 0.0,
        .kD = 0.0
    };

    constexpr double maxPilotInputRate = 5; // units per second, TODO: tune this value based on feel and testing
    constexpr double betaFilterN = 4; // filter time constant for the beta low pass filter used
    constexpr double betaZeroingMaxAuthority = 0.3; // max units that beta zeroing can contribute to the final yaw command
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

/**
 * @brief Filters the beta value with a simple low pass filter to be used for beta zeroing (sideslip correction).
 * 
 * @param beta Current raw beta value from state.
 * @param dt Current frame delta time.
 * @param previousFilteredBeta Previous frame filtered beta value.
 * 
 * @return Filtered beta value to be used for beta zeroing in the yaw command calculation.
 */
double Flight_Control_System::filterBeta(const double& beta, const double& dt, const double& previousFilteredBeta) {
    double alpha = dt / (betaFilterN + dt);
    return alpha * beta + (1 - alpha) * previousFilteredBeta;
}

/**
 * @brief Calculate the command yaw units value based off pilot input and aircraft state. Includes beta zeroing fusing with pilot input command
 * 
 * @param filteredInput Struct containing filtered pilot input values for the current frame.
 * @param aircraftState Struct containing current aircraft state values for the current frame.
 * @param limits Struct containing current axis limit values based on active FCS mode.
 * 
 * @return Unit value for yaw, post processing
*/
double Flight_Control_System::calculateYawCommand(const PilotInput& filteredInput, const AircraftState& aircraftState, const FcsLimits& limits) {
    double betaZeroingCommand = 0.0;

    // calculate beta zeroing (automated sideslip correction). 
    double filteredBeta = filterBeta(aircraftState.beta, m_dt, previousFilteredBeta);
    previousFilteredBeta = filteredBeta;
    clamp(filteredBeta, -betaZeroingMaxAuthority, betaZeroingMaxAuthority);

    if (std::abs(filteredBeta) > 0.01) // sideslip deadzone
    {
        betaZeroingCommand = betaZeroPValues.kP * -filteredBeta;
    }

    // calculate the yaw command for the pilots input
    double pilotCommandedYaw = filteredInput.yaw;

    // clamp the commanded yaw to stay within the max yaw rate based off law and dt
    pilotCommandedYaw = clamp(pilotCommandedYaw, -limits.maxYawRate * m_dt, limits.maxYawRate * m_dt);

    // calculate the final commanded yaw by fusing the beta zeroing with the pilot's filtered yaw input command. clamp to units
    return clamp(pilotCommandedYaw + betaZeroingCommand, -1.0, 1.0);
}

/**
 * @brief Calculate the commanded roll rate value based off pilot input and aircraft state.
 * 
 * @param filteredInput Struct containing filtered pilot input values for the current frame.
 * @param limits Struct containing current axis limit values based on active FCS mode.
 * 
 * @return Unit value for roll, post processing
 */
double Flight_Control_System::calculateRollCommand(const PilotInput& filteredInput, const AircraftState& aircraftState, const FcsLimits& limits) {
    // scale roll rate
    double goalRollRate = filteredInput.roll * limits.maxRollRate;

    //rate damping using a P controller
    double rollRateCorrection = rollRatePValues.kP * (goalRollRate - aircraftState.rollRate);

    // combine scaled and correction and clamp to max roll rate based on law and dt
    double commandedRollRate = goalRollRate + rollRateCorrection;
    commandedRollRate = clamp(commandedRollRate, -limits.maxRollRate, limits.maxRollRate);

    //return command normalized to units
    return clamp(commandedRollRate / limits.maxRollRate, -1.0, 1.0);
}

double Flight_Control_System::calculatePitchCommand(const PilotInput& filteredInput, const AircraftState& aircraftState, const FcsLimits& limits) {
    double goalPitchRate = filteredInput.pitch * limits.maxPitchRate;

    //rate damping using a P controller
    double pitchRateCorrection = pitchRatePValues.kP * (goalPitchRate - aircraftState.pitchRate);

    // g load protection
    double pitchG = pitchLimiterPValues.kP * clamp(limits.maxG - aircraftState.g, -limits.pitchAuthority, 0);

    // aoa protection
    double pitchAoa = pitchLimiterPValues.kP * clamp(limits.maxAoa - aircraftState.aoa, -limits.pitchAuthority, 0);

    // most limiting of g and aoa limiters wins
    double pitchProtection = std::min(pitchG, pitchAoa);

    // combine and clamp
    double commandedPitchRate = goalPitchRate + pitchRateCorrection + pitchProtection;
    commandedPitchRate = clamp(commandedPitchRate, -limits.maxPitchRate, limits.maxPitchRate);
    return clamp(commandedPitchRate / limits.maxPitchRate, -1.0, 1.0);
}

/**
 * @brief Smooth step function to give the pilot a feeling of resistance as they approach the limits, 
 * instead of an abrupt cutoff at the limit.
 * 
 * @param x Input value to be smoothed, expected in the range [-1, 1]. Values outside this range will be clamped
 * 
 * @return Smoothed output value in the range [-1, 1]
 */
double smoothStep(const double& x) {
    clamp(x, -1.0, 1.0);
    return x * x * (3 - 2 * x);
}

// Main per-frame FCS update. Order matters: mode selection -> axis limiters -> actuator helpers.
void Flight_Control_System::update(double dt)
{
    m_dt = dt;

    // Update current aircraft state struct
    currentAircraftState = {
        .g = m_state.getNY(),
        .aoa = m_state.m_aoa,
        .beta = m_state.m_beta,
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

    // Filter pilot input with rate limiters to prevent abrupt changes in commanded values that can cause instability
    pilotInputFiltered = filterPilotInput(pilotInputRaw, pilotInputPrev);

    // Decide active FCS mode based on current aircraft state; then update limits
    currentLaw = selectFcsLaw(currentAircraftState);
    currentLimits = updateLimitsForLaw(currentLaw);

    // Based on the currently selected law and filtered pilot input, calculate the appropriate commanded inputs
    commandedYaw = calculateYawCommand(pilotInputFiltered, currentAircraftState, currentLimits);
    commandedPitch = calculatePitchCommand(pilotInputFiltered, currentAircraftState, currentLimits);
    commandedRoll = calculateRollCommand(pilotInputFiltered, currentAircraftState, currentLimits);

    // Normalize/bound the filtered inputs in [-1, 1] using a smooth step equation to prevent over commanding, and to
    // give the pilot a feeling of resistance near the limits (prevents abrupt control cutoff with no sense of where limits are)
    // TODO: all

    // Take the smooth step results and run those through the pid loops (yaw, pitch, roll) and damper (yaw)
    // TODO: all

    // Hard clamp the pid outputs to [-1, 1] in case it goes out of bounds, and send a warning if it exceeds the bounds
    // TODO: all
}