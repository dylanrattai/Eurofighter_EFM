#include <gtest/gtest.h>
#include "../FCS.h"

// Aircraft specs for calculating expected test conditions
namespace {
    constexpr double DEG_TO_RAD { 3.14159265358979323846 / 180.0 };
    constexpr double RAD_TO_DEG { 180.0 / 3.14159265358979323846 };

    struct STRUCTURAL_LIMITS {
        // AOA Limits
        static constexpr double MAX_POSITIVE_AOA_DEG { 0.0 };
        static constexpr double MAX_NEGATIVE_AOA_DEG { 0.0 };

        // G Limits
        static constexpr double MAX_POSITIVE_G { 7.0 };
        static constexpr double MAX_NEGATIVE_G { -3.0 };

        // Pitch Rate Limits
        static constexpr double MAX_POSITIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };

        // Roll Rate Limits
        static constexpr double MAX_POSITIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
    };

    struct SUPERSONIC_LAW {
        // AOA Limits
        static constexpr double MAX_POSITIVE_AOA_DEG { 0.0 };
        static constexpr double MAX_NEGATIVE_AOA_DEG { 0.0 };

        // G Limits
        static constexpr double MAX_POSITIVE_G { 0.0 };
        static constexpr double MAX_NEGATIVE_G { 0.0 };

        // Pitch Rate Limits
        static constexpr double MAX_POSITIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };

        // Roll Rate Limits
        static constexpr double MAX_POSITIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
    };

    struct SUBSONIC_LAW {
        // AOA Limits
        static constexpr double MAX_POSITIVE_AOA_DEG { 0.0 };
        static constexpr double MAX_NEGATIVE_AOA_DEG { 0.0 };

        // G Limits
        static constexpr double MAX_POSITIVE_G { 0.0 };
        static constexpr double MAX_NEGATIVE_G { 0.0 };

        // Pitch Rate Limits
        static constexpr double MAX_POSITIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };

        // Roll Rate Limits
        static constexpr double MAX_POSITIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
    };

    struct AAR_LAW {
        // AOA Limits
        static constexpr double MAX_POSITIVE_AOA_DEG { 0.0 };
        static constexpr double MAX_NEGATIVE_AOA_DEG { 0.0 };

        // G Limits
        static constexpr double MAX_POSITIVE_G { 0.0 };
        static constexpr double MAX_NEGATIVE_G { 0.0 };

        // Pitch Rate Limits
        static constexpr double MAX_POSITIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };

        // Roll Rate Limits
        static constexpr double MAX_POSITIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
    };

    struct LANDING_LAW {
        // AOA Limits
        static constexpr double MAX_POSITIVE_AOA_DEG { 0.0 };
        static constexpr double MAX_NEGATIVE_AOA_DEG { 0.0 };

        // G Limits
        static constexpr double MAX_POSITIVE_G { 0.0 };
        static constexpr double MAX_NEGATIVE_G { 0.0 };

        // Pitch Rate Limits
        static constexpr double MAX_POSITIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_PITCH_RATE_DEG_PER_SEC { 0.0 };

        // Roll Rate Limits
        static constexpr double MAX_POSITIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
        static constexpr double MAX_NEGATIVE_ROLL_RATE_DEG_PER_SEC { 0.0 };
    };


}