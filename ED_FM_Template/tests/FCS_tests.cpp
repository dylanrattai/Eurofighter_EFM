#include <gtest/gtest.h>
#include "../FCS.h"
#include "FCS_test_fixture.h"
#define UNIT_TEST

/**
 * @file FCS_tests.cpp
 * @brief Unit tests for the Flight Control System (FCS) class, using Google Test framework.
 * @author Dylan Rattai
*/

// Aircraft specs for calculating expected test conditions
namespace {
    constexpr double DEG_TO_RAD { 3.14159265358979323846 / 180.0 };
    constexpr double RAD_TO_DEG { 180.0 / 3.14159265358979323846 };

    /**
     * @brief Struct to hold FCS limits for different modes, to be used in tests for expected values.
     */
    struct Limits {
        // AOA Limits
        double maxPositiveAoADeg;
        double maxNegativeAoADeg;

        // G Limits
        double maxPositiveG;
        double maxNegativeG;

        // Pitch Rate Limits
        double maxPositivePitchRateDegPerSec;
        double maxNegativePitchRateDegPerSec;

        // Roll Rate Limits
        double maxPositiveRollRateDegPerSec;
        double maxNegativeRollRateDegPerSec;
    };

    /**
     * @brief FCS limits for the structural limits of the aircraft.
     * 
     * @attention TODO: THESE ARE PLACEHOLDER VALUES, ALL 0.0
     */
    constexpr Limits STRUCTURAL_LIMITS {
        // AOA Limits
        .maxPositiveAoADeg { 0.0 },
        .maxNegativeAoADeg { 0.0 },

        // G Limits
        .maxPositiveG { 0.0 },
        .maxNegativeG { 0.0 },

        // Pitch Rate Limits
        .maxPositivePitchRateDegPerSec { 0.0 },
        .maxNegativePitchRateDegPerSec { 0.0 },

        // Roll Rate Limits
        .maxPositiveRollRateDegPerSec { 0.0 },
        .maxNegativeRollRateDegPerSec { 0.0 }
    };

    /**
     * @brief FCS limit consts for Air to Air refueling law.
     * 
     * @attention TODO: THESE ARE PLACEHOLDER VALUES, ALL 0.0
     */
    constexpr Limits REFUELING_LIMITS {
        // AOA Limits
        .maxPositiveAoADeg { 0.0 },
        .maxNegativeAoADeg { 0.0 },

        // G Limits
        .maxPositiveG { 0.0 },
        .maxNegativeG { 0.0 },

        // Pitch Rate Limits
        .maxPositivePitchRateDegPerSec { 0.0 },
        .maxNegativePitchRateDegPerSec { 0.0 },

        // Roll Rate Limits
        .maxPositiveRollRateDegPerSec { 0.0 },
        .maxNegativeRollRateDegPerSec { 0.0 }
    };

    /**
     * @brief FCS limit consts for landing law.
     * 
     * @attention TODO: THESE ARE PLACEHOLDER VALUES, ALL 0.0
     */
    constexpr Limits LANDING_LIMITS {
        // AOA Limits
        .maxPositiveAoADeg { 0.0 },
        .maxNegativeAoADeg { 0.0 },

        // G Limits
        .maxPositiveG { 0.0 },
        .maxNegativeG { 0.0 },

        // Pitch Rate Limits
        .maxPositivePitchRateDegPerSec { 0.0 },
        .maxNegativePitchRateDegPerSec { 0.0 },

        // Roll Rate Limits
        .maxPositiveRollRateDegPerSec { 0.0 },
        .maxNegativeRollRateDegPerSec { 0.0 }
    };

    /**
     * @brief FCS limit consts for supersonic law.
     * 
     * @attention TODO: THESE ARE PLACEHOLDER VALUES, ALL 0.0
     */
    constexpr Limits SUBSONIC_LIMITS {
        // AOA Limits
        .maxPositiveAoADeg { 0.0 },
        .maxNegativeAoADeg { 0.0 },

        // G Limits
        .maxPositiveG { 0.0 },
        .maxNegativeG { 0.0 },

        // Pitch Rate Limits
        .maxPositivePitchRateDegPerSec { 0.0 },
        .maxNegativePitchRateDegPerSec { 0.0 },

        // Roll Rate Limits
        .maxPositiveRollRateDegPerSec { 0.0 },
        .maxNegativeRollRateDegPerSec { 0.0 }
    };

    /**
     * @brief FCS limit consts for supersonic law.
     * 
     * @attention TODO: THESE ARE PLACEHOLDER VALUES, ALL 0.0
     */
    constexpr Limits SUPERSONIC_LIMITS {
        // AOA Limits
        .maxPositiveAoADeg { 0.0 },
        .maxNegativeAoADeg { 0.0 },

        // G Limits
        .maxPositiveG { 0.0 },
        .maxNegativeG { 0.0 },

        // Pitch Rate Limits
        .maxPositivePitchRateDegPerSec { 0.0 },
        .maxNegativePitchRateDegPerSec { 0.0 },

        // Roll Rate Limits
        .maxPositiveRollRateDegPerSec { 0.0 },
        .maxNegativeRollRateDegPerSec { 0.0 }
    };

    /**
     * @category FCS Mode Tests
    */

    /**
     * @brief Test that when the refueling door is fully open, the FCS mode is in refueling mode.
    */
    TEST_F(FCSTestFixture, LimiterModeRefuelingTrue) {
        setRefuelingDoorState(1.0);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  1.0);
        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(),   0.0);
    }

    /**
     * @brief Test that when the refueling door is just above the open threshold, the FCS mode is in refueling mode.
    */
    TEST_F(FCSTestFixture, LimiterModeRefuelingEdgeTrue) {
        setRefuelingDoorState(0.3);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  1.0);
        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(),   0.0);
    }

    /**
     * @brief Test that when the refueling door is just below the open threshold, the FCS mode is not in refueling mode.
    */
    TEST_F(FCSTestFixture, LimiterModeRefuelingEdgeFalse) {
        setRefuelingDoorState(0.29);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
    }

    /**
     * @brief Test that when the refueling door is closed, the FCS mode is not in refueling mode.
    */
    TEST_F(FCSTestFixture, LimiterModeRefuelingFalse) {
        setRefuelingDoorState(0.0);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
    }

    /**
     * @brief Test that when the gear is fully extended, the FCS mode is in landing mode.
    */
    TEST_F(FCSTestFixture, LimiterModeLandingTrue) {
        setGearNPosition(1.0);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    1.0);
        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(),   0.0);
    }

    /**
     * @brief Test that when the gear is just above the landing threshold, the FCS mode is in landing mode.
    */
    TEST_F(FCSTestFixture, LimiterModeLandingEdgeTrue) {
        setGearNPosition(0.99);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    1.0);
        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(),   0.0);
    }

    /**
     * @brief Test that when the gear is just below the landing threshold, the FCS mode is not in landing mode.
    */
    TEST_F(FCSTestFixture, LimiterModeLandingEdgeFalse) {
        setGearNPosition(0.98);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
    }

    /**
     * @brief Test that when the gear is retracted, the FCS mode is not in landing mode.
    */
    TEST_F(FCSTestFixture, LimiterModeLandingFalse) {
        setGearNPosition(0.0);

        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
    }

    /**
     * @brief Test that when the Mach number is supersonic, the FCS mode is in supersonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSupersonicTrue) {
        setMachNumber(1.5);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 1.0);
        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(),   0.0);
    }

    /**
     * @brief Test that when the Mach number is just above the supersonic threshold, the FCS mode is in supersonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSupersonicEdgeTrue) {
        setMachNumber(.99);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 1.0);
        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(),   0.0);
    }

    /**
     * @brief Test that when the Mach number is just below the supersonic threshold, the FCS mode is not in supersonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSupersonicEdgeFalse) {
        setMachNumber(.98);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
    }

    /**
     * @brief Test that when the Mach number is subsonic, the FCS mode is not in supersonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSupersonicFalse) {
        setMachNumber(0.5);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
    }

    /**
     * @brief Test that when the Mach number is subsonic, the FCS mode is in subsonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSubsonicTrue) {
        setMachNumber(0.5);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(), 1.0);
        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
    }

    /**
     * @brief Test that when the Mach number is just below the supersonic threshold, the FCS mode is in subsonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSubsonicEdgeTrue) {
        setMachNumber(.98);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(), 1.0);
        EXPECT_DOUBLE_EQ(fcs_.getSupersonicFCSMode(), 0.0);
        EXPECT_DOUBLE_EQ(fcs_.getLandingFCSMode(),    0.0);
        EXPECT_DOUBLE_EQ(fcs_.getRefuelingFCSMode(),  0.0);
    }

    /**
     * @brief Test that when the Mach number is just above the supersonic threshold, the FCS mode is not in subsonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSubsonicEdgeFalse) {
        setMachNumber(.99);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(), 0.0);
    }

    /**
     * @brief Test that when the Mach number is supersonic, the FCS mode is not in subsonic mode.
    */
    TEST_F(FCSTestFixture, LimiterModeSubsonicFalse) {
        setMachNumber(1.5);
        
        stepFcs(0.2);

        EXPECT_DOUBLE_EQ(fcs_.getSubsonicFCSMode(), 0.0);
     }

     /**
      * @category Axis Limiter Tests
      * @todo Tests for limit_roll
     */

    /**
     * @todo Tests for limit_pitch
     */

    /**
     * @todo Tests for limit_yaw
     */

    /**
     * @category Canard Tests
     * @todo Tests for autoDriveCanardPosition
    */

    /**
     * @category FCS Mode Limit Tests
    */

    /**
     * @brief Test that the correct limits are set for subsonic mode.
    */
    TEST_F(FCSTestFixture, SubsonicLimits) {
        // Set conditions for subsonic mode: Mach < 0.99, gear retracted, refueling door closed
        setMachNumber(0.5);
        setGearNPosition(0.0);
        setRefuelingDoorState(0.0);

        // Step time so FCS updates
        stepFcs(0.2);

        // PITCH RATE LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxPitchRate(), SUBSONIC_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativePitchRate(), SUBSONIC_LIMITS.maxNegativeAoADeg * DEG_TO_RAD);

        // G LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxG(), SUBSONIC_LIMITS.maxPositiveG);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativeG(), SUBSONIC_LIMITS.maxNegativeG);

        // AOA LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxAoA(), SUBSONIC_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);

        // ROLL LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxRollRate(), SUBSONIC_LIMITS.maxPositiveRollRateDegPerSec * DEG_TO_RAD);
    }

    /**
     * @brief Test that the correct limits are set for landing mode.
    */
    TEST_F(FCSTestFixture, LandingLimits) {
        // Set conditions for landing mode: gear extended, refueling door closed
        setGearNPosition(1.0);
        setRefuelingDoorState(0.0);

        // Step time so FCS updates
        stepFcs(0.2);

        // PITCH RATE LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxPitchRate(), LANDING_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativePitchRate(), LANDING_LIMITS.maxNegativeAoADeg * DEG_TO_RAD);

        // G LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxG(), LANDING_LIMITS.maxPositiveG);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativeG(), LANDING_LIMITS.maxNegativeG);

        // AOA LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxAoA(), LANDING_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);

        // ROLL LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxRollRate(), LANDING_LIMITS.maxPositiveRollRateDegPerSec * DEG_TO_RAD);
    }

    /**
     * @brief Test that the correct limits are set for supersonic mode.
    */
    TEST_F(FCSTestFixture, SupersonicLimits) {
        // Set conditions for supersonic mode: Mach > 0.99, gear retracted, refueling door closed
        setMachNumber(1.5);
        setGearNPosition(0.0);
        setRefuelingDoorState(0.0);

        // Step time so FCS updates
        stepFcs(0.2);

        // PITCH RATE LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxPitchRate(), SUPERSONIC_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativePitchRate(), SUPERSONIC_LIMITS.maxNegativeAoADeg * DEG_TO_RAD);

        // G LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxG(), SUPERSONIC_LIMITS.maxPositiveG);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativeG(), SUPERSONIC_LIMITS.maxNegativeG);

        // AOA LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxAoA(), SUPERSONIC_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);

        // ROLL LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxRollRate(), SUPERSONIC_LIMITS.maxPositiveRollRateDegPerSec * DEG_TO_RAD);
    }

    /**
     * @brief Test that the correct limits are set for refueling mode.
    */
    TEST_F(FCSTestFixture, RefuelingLimits) {
        // Set conditions for refueling mode: refueling door open
        setRefuelingDoorState(1.0);

        // Step time so FCS updates
        stepFcs(0.2);

        // PITCH RATE LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxPitchRate(), REFUELING_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativePitchRate(), REFUELING_LIMITS.maxNegativeAoADeg * DEG_TO_RAD);

        // G LIMITS
        EXPECT_DOUBLE_EQ(fcs_.getMaxG(), REFUELING_LIMITS.maxPositiveG);
        EXPECT_DOUBLE_EQ(fcs_.getMaxNegativeG(), REFUELING_LIMITS.maxNegativeG);

        // AOA LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxAoA(), REFUELING_LIMITS.maxPositiveAoADeg * DEG_TO_RAD);

        // ROLL LIMIT
        EXPECT_DOUBLE_EQ(fcs_.getMaxRollRate(), REFUELING_LIMITS.maxPositiveRollRateDegPerSec * DEG_TO_RAD);
    }
}