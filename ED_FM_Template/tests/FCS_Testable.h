#include <gtest/gtest.h>
#include "../FCS.h"

// Add this to the private section of Flight_Control_System.h:
//   friend class FCS_Testable;
class FCS_Testable : public Flight_Control_System
{
public:
    using Flight_Control_System::Flight_Control_System;

    FcsLaw     expSelectFcsLaw  (const AircraftState& s)                                               { return selectFcsLaw(s); }
    FcsLimits  expUpdateLimits  (const FcsLaw& law)                                                    { return updateLimitsForLaw(law); }
    PilotInput expFilterInput   (const PilotInput& raw, PilotInput& prev)                              { return filterPilotInput(raw, prev); }
    double     expCalcYaw       (const PilotInput& fi, const AircraftState& as, const FcsLimits& lim)  { return calculateYawCommand(fi, as, lim); }
    double     expCalcRoll      (const PilotInput& fi, const AircraftState& as, const FcsLimits& lim)  { return calculateRollCommand(fi, as, lim); }
    double     expCalcPitch     (const PilotInput& fi, const AircraftState& as, const FcsLimits& lim)  { return calculatePitchCommand(fi, as, lim); }
    double     expFilterBeta    (double beta, double dt, double prevBeta)                               { return filterBeta(beta, dt, prevBeta); }
};

class FlightControlSystemTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        fcs = std::make_unique<FCS_Testable>(m_state, m_input, m_airframe);
    }

    void TearDown() override
    {
        fcs.reset();
    }

    static Flight_Control_System::PilotInput makePilotInput(
        double pitch = 0.0, double roll = 0.0, double yaw = 0.0,
        double throttle1 = 0.0, double throttle2 = 0.0)
    {
        Flight_Control_System::PilotInput p{};
        p.pitch = pitch; p.roll = roll; p.yaw = yaw;
        p.throttle1 = throttle1; p.throttle2 = throttle2;
        return p;
    }

    static Flight_Control_System::AircraftState makeAircraftState(
        double g = 1.0, double aoa = 0.0, double beta = 0.0, double mach = 0.3,
        double pitchRate = 0.0, double rollRate = 0.0, double yawRate = 0.0,
        double nosewheelAngle = 0.0, double refuelingDoorPose = 0.0)
    {
        Flight_Control_System::AircraftState s{};
        s.g = g; s.aoa = aoa; s.beta = beta; s.mach = mach;
        s.pitchRate = pitchRate; s.rollRate = rollRate; s.yawRate = yawRate;
        s.nosewheelAngle = nosewheelAngle; s.refuelingDoorPose = refuelingDoorPose;
        return s;
    }

    static Flight_Control_System::FcsLimits makeFcsLimits(
        double maxG = 0.0, double maxAoa = 0.0, double maxRollRate = 0.0,
        double maxYawRate = 0.0, double maxPitchRate = 0.0)
    {
        Flight_Control_System::FcsLimits lim{};
        lim.maxG = maxG; lim.maxAoa = maxAoa; lim.maxRollRate = maxRollRate;
        lim.maxYawRate = maxYawRate; lim.maxPitchRate = maxPitchRate;
        return lim;
    }

    // Declaration order matters — each object is constructed top-to-bottom,
    // so dependencies must appear before the objects that reference them.
    State    m_state{};
    Input    m_input{};
    Engine   m_engine{m_state, m_input};
    Airframe m_airframe{m_state, m_input, m_engine};

    std::unique_ptr<FCS_Testable> fcs;
};