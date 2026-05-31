#pragma once
#include "Vec3.h"
#include "State.h"
#include "Input.h"
#include "Airframe.h"
#include "BaseComponent.h"
#include "Pid.h"
#include "Maths.h"

class Flight_Control_System
{
public:
    Flight_Control_System(State& state, Input& input, Airframe& airframe);

    virtual void zeroInit();
    virtual void coldInit();
    virtual void hotInit();
    virtual void airborneInit();

    void update(double dt);

    PID pitchController;
    PID rollController;

    enum class FcsLaw
    {
        SUBSONIC_LAW,
        SUPERSONIC_LAW,
        LANDING_LAW,
        REFUELING_LAW
    };

    struct AircraftState
    {
        double g;
        double aoa;
        double beta;
        double mach;
        double pitchRate;
        double rollRate;
        double yawRate;
        double nosewheelAngle;
        double refuelingDoorPose;
    };

    struct PilotInput
    {
        double pitch;
        double roll;
        double yaw;
        double throttle1;
        double throttle2;
    };

    struct FcsLimits
    {
        double maxG;
        double maxAoa;
        double maxRollRate;
        double maxYawRate;
        double maxPitchRate;
        double pitchAuthority;
    };

    struct PidValues
    {
        double kP;
        double kI;
        double kD;
    };

// For testing purposes, expose internal state of FCS modes and limits.
protected:
    

private:
    friend class FCS_Testable;

    FcsLaw currentLaw = FcsLaw::SUBSONIC_LAW;
    PilotInput pilotInputRaw{};
    PilotInput pilotInputPrev{};
    PilotInput pilotInputFiltered{};
    AircraftState currentAircraftState{};
    FcsLimits currentLimits{};

    double commandedYaw;
    double commandedRoll;
    double commandedPitch;

    PilotInput filterPilotInput(const PilotInput& rawInput, PilotInput& prevFilteredInput);
    FcsLaw selectFcsLaw(const AircraftState& state);
    FcsLimits updateLimitsForLaw(const FcsLaw& law);
    double calculateYawCommand(const PilotInput& filteredInput, const AircraftState& aircraftState, const FcsLimits& limits);
    double calculateRollCommand(const PilotInput& filteredInput, const AircraftState& aircraftState, const FcsLimits& limits);
    double calculatePitchCommand(const PilotInput& filteredInput, const AircraftState& aircraftState, const FcsLimits& limits);
    double filterBeta(const double& beta, const double& dt, const double& previousFilteredBeta);
    double smoothStep(const double& x);
    double pidLoopRoll(const double& rollCommand, const AircraftState& aircraftState, const FcsLimits& limits);
    double pidLoopPitch(const double& pitchCommand, const AircraftState& aircraftState, const FcsLimits& limits);
    double damperLoopYaw(const double& yawCommand, const AircraftState& aircraftState, const FcsLimits& limits);

    // ----- Class-scope physical constants -----
    static constexpr double DEG_TO_RAD        = 3.14159265358979323846 / 180.0;
    static constexpr double AOA_BUFFER_ZONE   = 0.05;   // ~3 deg, soft-limit blend zone
    static constexpr double BLEND_RATE       = 0.5;    // blend rate for soft limit zone
    static constexpr double GROUND_AOA_MAX    = 0.35;   // ~20 deg
    static constexpr double GROUND_AOA_MIN    = -0.09;  // ~-5 deg
    static constexpr double LANDING_AOA_LIMIT = 0.3142; // ~18 deg
    static constexpr double DEFAULT_AOA_LIMIT = 0.4189; // ~24 deg

    // ----- External system references -----
    State&    m_state;
    Input&    m_input;
    Airframe& m_airframe;

    double m_dt = 0.0;

    double previousFilteredBeta = 0.0;

    // ----- Canard animation state -----
    double newCanardAnims = 0.0;
    double canardPosition = 0.0;
};