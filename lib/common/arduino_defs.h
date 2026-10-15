/******************************************************************************
 * @file   arduino_defs.h
 * @brief  Common definitions for devices managed by LIBRA arduinos; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstdint>
#include <string>

// Other Library Headers
#include <QString>  // Qt::Core

/* --- TABLE OF CONTENTS ---
 * !Water Constants
 * !Water Helpers
 * !Flow Constants
 * !Flow Helpers
 * !Manip Constants
 * !Manip Helpers
 */

/**
 * @brief Common definitions for controlling the WaterArduino.
 */
namespace Water {

//------------------------------------------------------------------------------
// !Water Constants
//------------------------------------------------------------------------------

/**
 * @brief Fluid system side (see individual constants for description).
 */
enum class Side : uint8_t {
    kA = 0,  // LIBRA-I: left, LIBRA-II: only one
    kB,      // LIBRA-I: right
    kAll     // keep this last!
};

/**
 * @brief Fluid system state.
 */
enum class State : uint8_t { kStopped = 0, kFilling, kDraining };

// Per-setup Fluid Flow
#if LIBRA_VERSION == 1
// - 2*(3L + 6L + 10L)

constexpr double kFillRate = 2 * 0.129;   // L/s, TODO: measure w/ flow sensors
constexpr double kDrainRate = 2 * 0.061;  // L/s, TODO: measure w/ flow sensors
constexpr double kCapacity = 38;          // L
#elif LIBRA_VERSION == 2
// - 3L + 6L + 10L

constexpr double kFillRate = 0.129;   // L/s, TODO: measure w/ flow sensors
constexpr double kDrainRate = 0.061;  // L/s, TODO: measure w/ flow sensors
constexpr double kCapacity = 19;      // L
#endif

//------------------------------------------------------------------------------
// !Water Helpers
//------------------------------------------------------------------------------

/**
 * @brief Converts fluid system `Side` enum value to an equivalent string.
 *
 * @param side Fluid system side
 *
 * @note Only really useful for debugging.
 */
inline std::string SideEnumToString(const Side& side) {
    switch (side) {
        case Side::kA:
            return "A";
        case Side::kB:
            return "B";
        default:
            return "Undefined";
    }
}

/**
 * @brief Converts fluid system `State` enum value to a human-readable rich text
 *        (HTML) string.
 *
 * @param state Fluid system state
 */
inline QString StateEnumToString(const State& state) {
    static const QString no_text = "<span style='color: gray;'>--</span>";
    static const QString in_text = "<span style='color: green;'>IN</span>";
    static const QString out_text = "<span style='color: red;'>OUT</span>";

    switch (state) {
        case State::kStopped:
            return no_text;
        case State::kFilling:
            return in_text;
        case State::kDraining:
            return out_text;
        default:
            return "Undefined";
    }
}

}  // namespace Water

/**
 * @brief Common definitions for controlling the FlowArduino.
 */
namespace Flow {

//------------------------------------------------------------------------------
// !Flow Constants
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Flow Helpers
//------------------------------------------------------------------------------

}  // namespace Flow

#if LIBRA_VERSION == 1
/**
 * @brief Common definitions for controlling the ManipArduino.
 */
namespace Manip {

//------------------------------------------------------------------------------
// !Manip Constants
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Manip Helpers
//------------------------------------------------------------------------------

}  // namespace Manip
#endif
