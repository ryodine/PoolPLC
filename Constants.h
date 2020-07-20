/**
 * @file Constants.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Adjustable constants for the whole system
 * @version 0.1
 * @date 2020-07-19
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef PIN_MAPPINGS_GUARD_H
#define PIN_MAPPINGS_GUARD_H

#include <Controllino.h>

#define PIN_CAST(pin) static_cast<unsigned char>((pin))

namespace Constants {
namespace Algorithm {
//! The value past which any rotation is not acceptable
constexpr double k_correctTiltAtDegrees = 0.1;

//! The value/deadband which the algorithm should not attempt to correct
//! deviations within
constexpr double k_stopCorrectingTiltAtDegrees = 0.05;

//! The alpha for exponentially weighted average smoothing on the inclinometer
//! (higher = less smoothing)
constexpr double k_inclinometerEWMASmoothingAlpha = 0.5;
} // namespace Algorithm

namespace Physical {
//! This value represents the yaw offset of the installed sensor, which cannot
//! be determined automatically.
constexpr double k_inclinometerInstalledYawAdjustment = 0.0;
} // namespace Physical

namespace Pins {
// ========= BUTTON INPUTS ========= //
enum class BUTTON {
    ZERO = CONTROLLINO_A0,
    REFLASH_ACEINNA = CONTROLLINO_A3,
    RAISE = CONTROLLINO_A1,
    LOWER = CONTROLLINO_A2,
    CLEAR_FAULT = CONTROLLINO_A4
};
// ================================= //

// ======== CONTROL OUTPUTS ======== //
enum class RAM {
    RAISE_1 = CONTROLLINO_D0,
    LOWER_1 = CONTROLLINO_D1,
    RAISE_2 = CONTROLLINO_D2,
    LOWER_2 = CONTROLLINO_D3,
    RAISE_3 = CONTROLLINO_D4,
    LOWER_3 = CONTROLLINO_D5,
    RAISE_4 = CONTROLLINO_D6,
    LOWER_4 = CONTROLLINO_D7
};
enum class MOTOR {
    ENABLE_RAISE = CONTROLLINO_D8,
    ENABLE_LOWER = CONTROLLINO_D9
};
// ================================= //

// ======= INDICATOR OUTPUTS ======= //
enum class INDICATOR {
    FAULT_ACTIVE = CONTROLLINO_D12,
    FAULT_CLEARABLE = CONTROLLINO_D11,
    READY = CONTROLLINO_D10
};
// ================================= //

} // namespace Pins
} // namespace Constants

#endif // PIN_MAPPINGS_GUARD_H