#include <gtest/gtest.h>
#include "FCS_Testable.h"


/**
 * Test FCS refueling law selection based on refueling door pose. 
 * Refueling law should be selected when refueling door pose is above 0.1 (10% open), and not selected when below that threshold.
 */
TEST_F(FlightControlSystemTest, TestRefuelingLawTrue)
{
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.5);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::REFUELING_LAW);
}
TEST_F(FlightControlSystemTest, TestRefuelingLawFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.05);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::REFUELING_LAW);
}
TEST_F(FlightControlSystemTest, TestRefuelingLawEdgeTrue) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.1);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::REFUELING_LAW);
}
TEST_F(FlightControlSystemTest, TestRefuelingLawEdgeFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.099);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::REFUELING_LAW);
}

/**
 * Test FCS landing law selection based on nosewheel angle.
 * Landing law should be selected when nosewheel angle is above 0.1 (10% deflection), and not selected when below that threshold.
 */
TEST_F(FlightControlSystemTest, TestLandingLawTrue) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.5, 0.0);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::LANDING_LAW);
}
TEST_F(FlightControlSystemTest, TestLandingLawFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.05, 0.0);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::LANDING_LAW);
}
TEST_F(FlightControlSystemTest, TestLandingLawEdgeTrue) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.1, 0.0);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::LANDING_LAW);
}
TEST_F(FlightControlSystemTest, TestLandingLawEdgeFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.099, 0.0);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::LANDING_LAW);
}

/**
 * Test FCS supersonic law selection based on Mach number.
 * Supersonic law should be selected when Mach is above or equal to 1.0, and not selected when below that threshold.
 */
TEST_F(FlightControlSystemTest, TestSupersonicLawTrue) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUPERSONIC_LAW);
}
TEST_F(FlightControlSystemTest, TestSupersonicLawFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUPERSONIC_LAW);
}
TEST_F(FlightControlSystemTest, TestSupersonicLawEdgeTrue) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUPERSONIC_LAW);
}
TEST_F(FlightControlSystemTest, TestSupersonicLawEdgeFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.999, 0.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUPERSONIC_LAW);
}

/**
 * Test FCS subsonic law selection as the default law when no other mode selection conditions are met.
 * Subsonic law should be selected when refueling door pose is below 0.1, nosewheel angle is below 0.1, and Mach is below 1.0.
 */
TEST_F(FlightControlSystemTest, TestSubsonicLawTrue) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUBSONIC_LAW);
}
TEST_F(FlightControlSystemTest, TestSubsonicLawFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.5, 0.5);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUBSONIC_LAW);
}  
TEST_F(FlightControlSystemTest, TestSubsonicLawEdgeTrue) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 0.05);
    EXPECT_NE(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUBSONIC_LAW);
}
TEST_F(FlightControlSystemTest, TestSubsonicLawEdgeFalse) {
    auto state = makeAircraftState(1.0, 0.0, 0.0, 0.999, 0.0, 0.0, 0.0, 0.099, 0.099);
    EXPECT_EQ(fcs->expSelectFcsLaw(state), Flight_Control_System::FcsLaw::SUBSONIC_LAW);
}