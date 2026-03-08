#pragma once

#include <gtest/gtest.h>

// Test-only visibility seam to allow deterministic Airframe state setup.
#define private public
#include "../FCS.h"
#undef private

#include "../Engine.h"

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
