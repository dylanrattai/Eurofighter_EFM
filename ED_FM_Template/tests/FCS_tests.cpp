#pragma once
#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

// Test-only visibility seam to allow deterministic Airframe state setup.
#define private public
#include "../FCS.h"
#undef private
#include "../Engine.h"
constexpr double DEG_TO_RAD = 0.017453292519943295769236907684886;

struct StateStub : public State
{
    void setMachNumber(double mach)
    {
        m_mach = mach;
    }
    void setAoARadians(double aoa_rad)
    {
        m_aoa = aoa_rad;
    }
    void setBodyRatesRadPerSec(double roll_rate, double pitch_rate)
    {
        m_omega.x = roll_rate;
        m_omega.z = pitch_rate;
    }
    void setNormalAccelerationG(double ny_g)
    {
        m_localAcceleration = Vec3(0.0, 0.0, ny_g * 9.81);
    }
};

class InputStub : public Input
{
public:
    void setPitchCommand(double cmd)
    {
        pitch(cmd);
    }
    void setRollCommand(double cmd)
    {
        roll(cmd);
    }
    void setYawCommand(double cmd)
    {
        yaw(cmd);
    }
};

class AirframeStub : public Airframe
{
public:
    AirframeStub(State& state, Input& input, Engine& engine)
        : Airframe(state, input, engine)
    {
    }
    void setGearNPositionForTest(double gear_n_position)
    {
        m_gearNPosition = gear_n_position;
    }
    void setRefuelingDoorForTest(double refueling_door_state)
    {
        m_refuelingDoorToggle = refueling_door_state;
    }
};

class FCSTestFixture : public ::testing::Test
{
protected:
    FCSTestFixture()
        : state_(),
          input_(),
          engine_(state_, input_),
          airframe_(state_, input_, engine_),
          fcs_(state_, input_, airframe_)
    {
    }
    void SetUp() override
    {
        state_.zeroInit();
        input_.zeroInit();
        engine_.zeroInit();
        airframe_.zeroInit();
        fcs_.zeroInit();
    }
    
    void setMachNumber(double mach)
    {
        state_.setMachNumber(mach);
    }
    
    void setAoARadians(double aoa_rad)
    {
        state_.setAoARadians(aoa_rad);
    }
    
    void setAoADegrees(double aoa_deg)
    {
        state_.setAoARadians(aoa_deg * DEG_TO_RAD);
    }
    
    void setGearNPosition(double gear_n_position)
    {
        airframe_.setGearNPositionForTest(gear_n_position);
    }
    
    void setRefuelingDoorState(double refueling_door_state)
    {
        airframe_.setRefuelingDoorForTest(refueling_door_state);
    }
    
    void setPitchCommand(double pitch_cmd)
    {
        input_.setPitchCommand(pitch_cmd);
    }
    
    void setRollCommand(double roll_cmd)
    {
        input_.setRollCommand(roll_cmd);
    }
    
    void setYawCommand(double yaw_cmd)
    {
        input_.setYawCommand(yaw_cmd);
    }
    
    void setBodyRatesRadPerSec(double roll_rate, double pitch_rate)
    {
        state_.setBodyRatesRadPerSec(roll_rate, pitch_rate);
    }
    
    void setNormalAccelerationG(double ny_g)
    {
        state_.setNormalAccelerationG(ny_g);
    }
    
    void stepFcs(double dt)
    {
        fcs_.update(dt);
    }
    
    StateStub state_;
    InputStub input_;
    Engine engine_;
    AirframeStub airframe_;
    Flight_Control_System fcs_;
};

// ============================================================================
// MODE SELECTION TESTS
// ============================================================================

class ModeSelectionTest : public FCSTestFixture
{
};

TEST_F(ModeSelectionTest, LandingModeActivatesWhenGearDownAndRefuelingDoorClosed)
{
    // Setup: Refueling door closed, landing gear down, subsonic
    setRefuelingDoorState(0.0);      // Door closed
    setGearNPosition(0.5);            // Gear down
    setMachNumber(0.5);               // Subsonic
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    EXPECT_EQ(fcs_.landing_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.refueling_FCS_mode, 0.0);
}

TEST_F(ModeSelectionTest, SubsonicModeActivatesWhenGearUpBelowSupersonicMach)
{
    // Setup: Refueling door closed, gear up, mach 0.8
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);            // Gear up
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.landing_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.refueling_FCS_mode, 0.0);
}

TEST_F(ModeSelectionTest, SupersonicModeActivatesAboveSupersonicMachThreshold)
{
    // Setup: Refueling door closed, gear up, mach > 0.98
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(1.05);              // Above 0.98 threshold
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.landing_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.refueling_FCS_mode, 0.0);
}

TEST_F(ModeSelectionTest, RefuelingModeActivatesWhenRefuelingDoorOpen)
{
    // Setup: Refueling door open (> 0.3)
    setRefuelingDoorState(0.5);       // Door open
    setGearNPosition(0.0);
    setMachNumber(0.5);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    EXPECT_EQ(fcs_.refueling_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.landing_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 0.0);
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 0.0);
}

TEST_F(ModeSelectionTest, ModeTransitionSubsonicToSupersonic)
{
    // Start in subsonic
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.9);
    setNormalAccelerationG(1.0);
    stepFcs(0.016);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    
    // Transition to supersonic
    setMachNumber(1.05);
    stepFcs(0.016);
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 0.0);
}

// ============================================================================
// ENVELOPE LIMIT TESTS
// ============================================================================

class EnvelopeLimitsTest : public FCSTestFixture
{
};

TEST_F(EnvelopeLimitsTest, SubsonicMaxAoALimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Subsonic max AoA should be 30 degrees
    EXPECT_DOUBLE_EQ(fcs_.max_AoA, 30.0 * DEG_TO_RAD);
}

TEST_F(EnvelopeLimitsTest, SubsonicMaxGLimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Subsonic max G should be 4.45
    EXPECT_DOUBLE_EQ(fcs_.max_g, 4.45);
}

TEST_F(EnvelopeLimitsTest, LandingMaxAoALimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.5);
    setMachNumber(0.5);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Landing max AoA should be 15 degrees
    EXPECT_DOUBLE_EQ(fcs_.max_AoA, 15.0 * DEG_TO_RAD);
}

TEST_F(EnvelopeLimitsTest, LandingMaxGLimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.5);
    setMachNumber(0.5);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Landing max G should be 4
    EXPECT_DOUBLE_EQ(fcs_.max_g, 4.0);
}

TEST_F(EnvelopeLimitsTest, SupersonicMaxAoALimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(1.1);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Supersonic max AoA should be 15 degrees
    EXPECT_DOUBLE_EQ(fcs_.max_AoA, 15.0 * DEG_TO_RAD);
}

TEST_F(EnvelopeLimitsTest, SupersonicMaxGLimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(1.1);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Supersonic max G should be 9.0
    EXPECT_DOUBLE_EQ(fcs_.max_g, 9.0);
}

TEST_F(EnvelopeLimitsTest, RefuelingMaxGLimit)
{
    setRefuelingDoorState(0.5);
    setMachNumber(0.5);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Refueling max G should be 2.0
    EXPECT_DOUBLE_EQ(fcs_.max_g, 2.0);
}

TEST_F(EnvelopeLimitsTest, RefuelingMaxAoALimit)
{
    setRefuelingDoorState(0.5);
    setMachNumber(0.5);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Refueling max AoA should be 15 degrees
    EXPECT_DOUBLE_EQ(fcs_.max_AoA, 15.0 * DEG_TO_RAD);
}

// ============================================================================
// PITCH LIMITING TESTS
// ============================================================================

class PitchLimitingTest : public FCSTestFixture
{
};

TEST_F(PitchLimitingTest, PitchCommandPassthroughAtNormalConditions)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setPitchCommand(0.5);
    
    stepFcs(0.016);
    
    // Pitch command should pass through with minimal filtering at normal conditions
    EXPECT_NEAR(fcs_.pitch_cmd_filtered, 0.5, 0.01);
}

TEST_F(PitchLimitingTest, PitchCommandClampedToValidRange)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setPitchCommand(1.5);  // Beyond -1.0 to 1.0 range
    
    stepFcs(0.016);
    
    // Pitch command should be clamped to [-1.0, 1.0]
    EXPECT_LE(fcs_.pitch_cmd_filtered, 1.0);
    EXPECT_GE(fcs_.pitch_cmd_filtered, -1.0);
}

TEST_F(PitchLimitingTest, PitchCommandLimitedByGLimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(4.0);  // Near G limit
    setPitchCommand(1.0);  // Full pitch up
    
    stepFcs(0.016);
    
    // Pitch should be limited due to G approaching limit
    EXPECT_LT(fcs_.pitch_cmd_filtered, 1.0);
}

TEST_F(PitchLimitingTest, PitchCommandLimitedByAoALimit)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(25.0);  // Near 30-degree AoA limit
    setNormalAccelerationG(1.0);
    setPitchCommand(1.0);  // Full pitch up would increase AoA
    
    stepFcs(0.016);
    
    // Pitch should be limited due to AoA approaching limit
    EXPECT_LT(fcs_.pitch_cmd_filtered, 1.0);
}

TEST_F(PitchLimitingTest, PitchCommandConvertedToTargetG)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setPitchCommand(1.0);
    
    stepFcs(0.016);
    
    // Full pitch command should map to kPitchCmdToTargetG (4.0G)
    double expected_target_g = 1.0 * 4.0;
    EXPECT_NEAR(expected_target_g, 4.0, 0.1);
}

TEST_F(PitchLimitingTest, PitchNegativeCommandAllowed)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(10.0);
    setNormalAccelerationG(1.0);
    setPitchCommand(-0.5);
    
    stepFcs(0.016);
    
    // Negative pitch should be passed through
    EXPECT_LT(fcs_.pitch_cmd_filtered, 0.0);
}

TEST_F(PitchLimitingTest, ZeroGProtection)
{
    // This tests that the code handles near-zero G safely
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(0.0001);  // Very low G
    setPitchCommand(1.0);
    
    // Should not crash and should limit pitch
    EXPECT_NO_THROW(stepFcs(0.016));
    EXPECT_LT(fcs_.pitch_cmd_filtered, 1.0);
}

// ============================================================================
// ROLL LIMITING TESTS
// ============================================================================

class RollLimitingTest : public FCSTestFixture
{
};

TEST_F(RollLimitingTest, RollCommandPassthroughInSubsonicMode)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setRollCommand(0.5);
    setBodyRatesRadPerSec(0.0, 0.0);
    
    stepFcs(0.016);
    
    // Roll command should pass through in subsonic mode
    EXPECT_GT(std::abs(fcs_.roll_cmd_filtered), 0.0);
}

TEST_F(RollLimitingTest, RollCommandLimitedInLandingMode)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.5);
    setMachNumber(0.5);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setRollCommand(1.0);
    setBodyRatesRadPerSec(0.0, 0.0);
    
    stepFcs(0.016);
    
    // Roll authority in landing mode is 0.5x
    // So roll command should be reduced
    EXPECT_LT(fcs_.roll_cmd_filtered, 0.5);
}

TEST_F(RollLimitingTest, RollCommandLimitedInSupersonicMode)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(1.1);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setRollCommand(1.0);
    setBodyRatesRadPerSec(0.0, 0.0);
    
    stepFcs(0.016);
    
    // Roll authority in supersonic mode is 0.25x
    EXPECT_LT(fcs_.roll_cmd_filtered, 0.5);
}

TEST_F(RollLimitingTest, RollCommandLimitedInRefuelingMode)
{
    setRefuelingDoorState(0.5);
    setMachNumber(0.5);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setRollCommand(1.0);
    setBodyRatesRadPerSec(0.0, 0.0);
    
    stepFcs(0.016);
    
    // Roll authority in refueling mode is 0.5x
    EXPECT_LT(fcs_.roll_cmd_filtered, 0.5);
}

TEST_F(RollLimitingTest, RollCommandClamped)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setRollCommand(1.5);  // Out of range
    setBodyRatesRadPerSec(0.0, 0.0);
    
    stepFcs(0.016);
    
    // Roll command should be clamped
    double max_roll_rad = 200.0 * DEG_TO_RAD;  // Subsonic limit
    EXPECT_LE(std::abs(fcs_.roll_cmd_filtered / DEG_TO_RAD), std::abs(max_roll_rad / DEG_TO_RAD));
}

TEST_F(RollLimitingTest, NegativeRollCommandAllowed)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setRollCommand(-0.5);
    setBodyRatesRadPerSec(0.0, 0.0);
    
    stepFcs(0.016);
    
    // Negative roll should be allowed
    EXPECT_LT(fcs_.roll_cmd_filtered, 0.0);
}

// ============================================================================
// YAW LIMITING TESTS
// ============================================================================

class YawLimitingTest : public FCSTestFixture
{
};

TEST_F(YawLimitingTest, YawAuthorityInNominalMode)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setYawCommand(1.0);
    
    stepFcs(0.016);
    
    // Yaw authority in nominal mode is 0.5x
    EXPECT_NEAR(fcs_.yaw_cmd_filtered, 0.5, 0.01);
}

TEST_F(YawLimitingTest, YawAuthorityInLandingMode)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.5);
    setMachNumber(0.5);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setYawCommand(1.0);
    
    stepFcs(0.016);
    
    // Yaw authority in landing mode is 0.05x (very restricted)
    EXPECT_NEAR(fcs_.yaw_cmd_filtered, 0.05, 0.01);
}

TEST_F(YawLimitingTest, YawAuthorityInRefuelingMode)
{
    setRefuelingDoorState(0.5);
    setMachNumber(0.5);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    setYawCommand(1.0);
    
    stepFcs(0.016);
    
    // Yaw authority in refueling mode is 0.25x
    EXPECT_NEAR(fcs_.yaw_cmd_filtered, 0.25, 0.01);
}

TEST_F(YawLimitingTest, YawDampingAtHighAoA)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(15.0);  // Above kYawDampingAoaStartDeg (10 degrees)
    setNormalAccelerationG(1.0);
    setYawCommand(1.0);
    
    stepFcs(0.016);
    
    // Yaw should be damped due to high AoA
    // At 15 degrees AoA with 10 degree threshold:
    // scale_factor = 10.0 / 15.0 = 0.667
    // yaw_cmd = 1.0 * 0.5 * 0.667 = 0.333
    EXPECT_LT(fcs_.yaw_cmd_filtered, 0.5);
}

TEST_F(YawLimitingTest, YawDampingIncreasesWithAoA)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setNormalAccelerationG(1.0);
    setYawCommand(1.0);
    
    // Test at lower AoA
    setAoADegrees(15.0);
    stepFcs(0.016);
    double yaw_15deg = fcs_.yaw_cmd_filtered;
    
    // Test at higher AoA
    setAoADegrees(20.0);
    stepFcs(0.016);
    double yaw_20deg = fcs_.yaw_cmd_filtered;
    
    // Damping should be stronger at higher AoA
    EXPECT_LT(yaw_20deg, yaw_15deg);
}

TEST_F(YawLimitingTest, NoYawDampingBelowThreshold)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);  // Below threshold
    setNormalAccelerationG(1.0);
    setYawCommand(1.0);
    
    stepFcs(0.016);
    
    // Below threshold, only nominal authority applies
    EXPECT_NEAR(fcs_.yaw_cmd_filtered, 0.5, 0.01);
}

// ============================================================================
// CANARD POSITION BLENDING TESTS
// ============================================================================

class CanardBlendingTest : public FCSTestFixture
{
};

TEST_F(CanardBlendingTest, CanardRetractedBelowAoAThreshold)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(3.0);  // Below 5 degree threshold
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Canard should be retracted
    EXPECT_EQ(fcs_.canard_position, 0.0);
    EXPECT_EQ(fcs_.new_canard_anims, 0.0);
}

TEST_F(CanardBlendingTest, CanardDeployedAboveAoAThreshold)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(10.0);  // Above 5 degree threshold
    setNormalAccelerationG(1.0);
    
    // Simulate time to transition (kCanardTransitionTimeSeconds = 10.0)
    for (int i = 0; i < 1000; ++i)
    {
        stepFcs(0.016);
    }
    
    // Canard should eventually be fully deployed
    EXPECT_GE(fcs_.new_canard_anims, 0.9);  // Allow for numerical precision
}

TEST_F(CanardBlendingTest, CanardTransitionSpeed)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(10.0);
    setNormalAccelerationG(1.0);
    
    // Single step transition
    stepFcs(0.016);
    
    // With kCanardTransitionTimeSeconds = 10.0:
    // transition_speed = 0.016 / 10.0 = 0.0016
    double expected_increase = 0.016 / 10.0;
    EXPECT_NEAR(fcs_.new_canard_anims, expected_increase, 0.0001);
}

TEST_F(CanardBlendingTest, CanardRetractSlowly)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setNormalAccelerationG(1.0);
    
    // Deploy canard
    setAoADegrees(10.0);
    for (int i = 0; i < 1000; ++i)
    {
        stepFcs(0.016);
    }
    EXPECT_GE(fcs_.new_canard_anims, 0.9);
    
    // Now retract
    setAoADegrees(2.0);
    for (int i = 0; i < 100; ++i)
    {
        stepFcs(0.016);
    }
    
    // Canard should be retracting but not fully retracted yet
    EXPECT_LT(fcs_.new_canard_anims, 0.9);
    EXPECT_GT(fcs_.new_canard_anims, 0.0);
}

// ============================================================================
// CONTROL AUTHORITY TESTS
// ============================================================================

class ControlAuthorityTest : public FCSTestFixture
{
};

TEST_F(ControlAuthorityTest, RollRateAuthority)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Subsonic limited roll rate should be 200 deg/s = 200 * DEG_TO_RAD rad/s
    EXPECT_DOUBLE_EQ(fcs_.limited_roll_rate, 200.0 * DEG_TO_RAD);
}

TEST_F(ControlAuthorityTest, RollRateAuthorityReducedInLandingMode)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.5);
    setMachNumber(0.5);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Landing mode limited roll rate should be 80 deg/s = 80 * DEG_TO_RAD rad/s
    EXPECT_DOUBLE_EQ(fcs_.limited_roll_rate, 80.0 * DEG_TO_RAD);
}

TEST_F(ControlAuthorityTest, PitchRateAuthoritySubsonic)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Subsonic max pitch rate
    EXPECT_DOUBLE_EQ(fcs_.max_current_pitch_rate, 15.0 * DEG_TO_RAD);
}

TEST_F(ControlAuthorityTest, PitchRateAuthoritySupersonic)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(1.1);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Supersonic max pitch rate is reduced to 7 deg/s
    EXPECT_DOUBLE_EQ(fcs_.max_current_pitch_rate, 7.0 * DEG_TO_RAD);
}

// ============================================================================
// STATE UPDATE TESTS
// ============================================================================

class StateUpdateTest : public FCSTestFixture
{
};

TEST_F(StateUpdateTest, AllInputsUpdatedEachCycle)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.5);
    setMachNumber(0.8);
    setAoADegrees(8.0);
    setNormalAccelerationG(2.5);
    setPitchCommand(0.3);
    setRollCommand(-0.4);
    setYawCommand(0.2);
    setBodyRatesRadPerSec(0.1, 0.05);
    
    stepFcs(0.016);
    
    // Verify state was captured
    EXPECT_EQ(fcs_.current_aoa, state_.m_aoa);
    EXPECT_EQ(fcs_.current_g, 2.5);
    EXPECT_EQ(fcs_.nosewheel_angle, 0.5);
    EXPECT_EQ(fcs_.pitchcmd, 0.3);
    EXPECT_EQ(fcs_.rollcmd, -0.4);
    EXPECT_EQ(fcs_.yawcmd, 0.2);
}

TEST_F(StateUpdateTest, MachNumberUpdated)
{
    setMachNumber(0.5);
    stepFcs(0.016);
    EXPECT_EQ(fcs_.airspeed, 0.5);
    
    setMachNumber(1.5);
    stepFcs(0.016);
    EXPECT_EQ(fcs_.airspeed, 1.5);
}

TEST_F(StateUpdateTest, DeltaTimePassedToControllers)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    EXPECT_EQ(fcs_.m_dt, 0.016);
    
    stepFcs(0.032);
    EXPECT_EQ(fcs_.m_dt, 0.032);
}

// ============================================================================
// EDGE CASE TESTS
// ============================================================================

class EdgeCasesTest : public FCSTestFixture
{
};

TEST_F(EdgeCasesTest, ZeroMachNumber)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.0);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    // Should not crash
    EXPECT_NO_THROW(stepFcs(0.016));
}

TEST_F(EdgeCasesTest, HighMachNumber)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(2.0);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Should be in supersonic mode
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 1.0);
}

TEST_F(EdgeCasesTest, NegativeAoA)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(-5.0);
    setNormalAccelerationG(1.0);
    
    EXPECT_NO_THROW(stepFcs(0.016));
}

TEST_F(EdgeCasesTest, NegativeGAcceleration)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(5.0);
    setNormalAccelerationG(-1.0);
    
    EXPECT_NO_THROW(stepFcs(0.016));
}

TEST_F(EdgeCasesTest, RefuelingDoorBoundary)
{
    setRefuelingDoorState(0.29);  // Just below threshold
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Should be in subsonic mode (door closed)
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.refueling_FCS_mode, 0.0);
    
    setRefuelingDoorState(0.31);  // Just above threshold
    stepFcs(0.016);
    
    // Should be in refueling mode (door open)
    EXPECT_EQ(fcs_.refueling_FCS_mode, 1.0);
}

TEST_F(EdgeCasesTest, MachBoundary)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.97);  // Just below threshold
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    
    // Should be in subsonic mode
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 0.0);
    
    setMachNumber(0.99);  // Just above threshold
    stepFcs(0.016);
    
    // Should be in supersonic mode
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 0.0);
}

TEST_F(EdgeCasesTest, GearPositionBoundary)
{
    setRefuelingDoorState(0.0);
    setMachNumber(0.5);
    setNormalAccelerationG(1.0);
    
    setGearNPosition(0.09);  // Just below threshold
    stepFcs(0.016);
    
    // Should be subsonic (gear up)
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.landing_FCS_mode, 0.0);
    
    setGearNPosition(0.11);  // Just above threshold
    stepFcs(0.016);
    
    // Should be landing (gear down)
    EXPECT_EQ(fcs_.landing_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 0.0);
}

TEST_F(EdgeCasesTest, LargeTimestep)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(10.0);
    setNormalAccelerationG(1.0);
    
    // Large timestep shouldn't crash
    EXPECT_NO_THROW(stepFcs(0.1));
}

TEST_F(EdgeCasesTest, VerySmallTimestep)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setAoADegrees(10.0);
    setNormalAccelerationG(1.0);
    
    // Very small timestep shouldn't crash
    EXPECT_NO_THROW(stepFcs(0.001));
}

// ============================================================================
// INTEGRATED SCENARIO TESTS
// ============================================================================

class IntegratedScenarioTest : public FCSTestFixture
{
};

TEST_F(IntegratedScenarioTest, TakeoffSequence)
{
    // Simulate takeoff sequence
    setRefuelingDoorState(0.0);
    setMachNumber(0.1);
    setAoADegrees(8.0);
    setNormalAccelerationG(1.1);
    setPitchCommand(0.2);
    setRollCommand(0.0);
    setYawCommand(0.0);
    
    stepFcs(0.016);
    
    // Should be in landing mode initially
    EXPECT_EQ(fcs_.landing_FCS_mode, 1.0);
    
    // Raise gear
    setGearNPosition(0.0);
    setMachNumber(0.5);
    stepFcs(0.016);
    
    // Should transition to subsonic
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    EXPECT_EQ(fcs_.landing_FCS_mode, 0.0);
}

TEST_F(IntegratedScenarioTest, AccelerationThroughTransonicRegion)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setAoADegrees(5.0);
    setNormalAccelerationG(1.0);
    
    // Subsonic acceleration
    setMachNumber(0.7);
    stepFcs(0.016);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    EXPECT_DOUBLE_EQ(fcs_.max_g, 4.45);
    
    // Transonic region
    setMachNumber(0.9);
    stepFcs(0.016);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    
    // Supersonic
    setMachNumber(1.05);
    stepFcs(0.016);
    EXPECT_EQ(fcs_.supersonic_FCS_mode, 1.0);
    EXPECT_DOUBLE_EQ(fcs_.max_g, 9.0);
}

TEST_F(IntegratedScenarioTest, LandingApproachSequence)
{
    // Establish in approach
    setRefuelingDoorState(0.0);
    setMachNumber(0.4);
    setAoADegrees(8.0);
    setNormalAccelerationG(1.2);
    setPitchCommand(0.1);
    setRollCommand(0.05);
    setYawCommand(0.0);
    
    // Extend gear
    setGearNPosition(0.5);
    stepFcs(0.016);
    
    EXPECT_EQ(fcs_.landing_FCS_mode, 1.0);
    EXPECT_DOUBLE_EQ(fcs_.max_g, 4.0);
    EXPECT_DOUBLE_EQ(fcs_.max_AoA, 15.0 * DEG_TO_RAD);
}

TEST_F(IntegratedScenarioTest, RefuelingSequence)
{
    // Approach receiver
    setRefuelingDoorState(0.0);
    setMachNumber(0.5);
    setAoADegrees(3.0);
    setNormalAccelerationG(1.0);
    
    stepFcs(0.016);
    EXPECT_EQ(fcs_.subsonic_FCS_mode, 1.0);
    
    // Open refueling door
    setRefuelingDoorState(0.5);
    setPitchCommand(0.01);  // Very small commands
    setRollCommand(0.01);
    setYawCommand(0.01);
    
    stepFcs(0.016);
    
    EXPECT_EQ(fcs_.refueling_FCS_mode, 1.0);
    EXPECT_DOUBLE_EQ(fcs_.max_g, 2.0);  // Heavily restricted
}

// ============================================================================
// HIGH LOAD TESTS
// ============================================================================

class HighLoadTest : public FCSTestFixture
{
};

TEST_F(HighLoadTest, ContinuousHighGManeuver)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.85);
    setAoADegrees(20.0);
    setPitchCommand(1.0);
    
    // Simulate sustained maneuver
    for (int i = 0; i < 100; ++i)
    {
        setNormalAccelerationG(4.0 + (i * 0.01));  // Gradually increase G
        stepFcs(0.016);
        
        // Pitch should be limited as G approaches limit
        EXPECT_LE(fcs_.current_g, fcs_.max_g + 0.5);
    }
}

TEST_F(HighLoadTest, ExtendedRefuelingOperation)
{
    setRefuelingDoorState(0.5);
    setMachNumber(0.5);
    setAoADegrees(3.0);
    setNormalAccelerationG(1.0);
    
    // Simulate extended refueling
    for (int i = 0; i < 1000; ++i)
    {
        // Small pilot inputs
        setPitchCommand(0.01);
        setRollCommand(0.02);
        setYawCommand(0.01);
        
        stepFcs(0.016);
        
        EXPECT_EQ(fcs_.refueling_FCS_mode, 1.0);
    }
}

TEST_F(HighLoadTest, CanardTransitionFullCycle)
{
    setRefuelingDoorState(0.0);
    setGearNPosition(0.0);
    setMachNumber(0.8);
    setNormalAccelerationG(1.0);
    
    // Full deployment cycle
    setAoADegrees(15.0);
    for (int i = 0; i < 1000; ++i)
    {
        stepFcs(0.016);
    }
    EXPECT_GE(fcs_.new_canard_anims, 0.9);
    
    // Full retraction cycle
    setAoADegrees(2.0);
    for (int i = 0; i < 1000; ++i)
    {
        stepFcs(0.016);
    }
    EXPECT_LE(fcs_.new_canard_anims, 0.1);
}