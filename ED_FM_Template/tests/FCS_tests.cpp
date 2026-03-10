#include <gtest/gtest.h>
#include "../FCS.h"

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
}