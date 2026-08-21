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
 *
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

//------------------------------------------------------------------------------
// !Water Helpers
//------------------------------------------------------------------------------

/**
 * @brief Converts fluid system `Side` enum value to an equivalent string.
 *
 * @param side Fluid system side.
 * @return QString Human-readable string.
 *
 * @note Only really useful for debugging.
 */
inline std::string SideEnumToString(Side side) {
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
 * @brief Converts fluid system `State` enum value to an appropriately formatted
 *        string.
 *
 * @param state Fluid system state.
 * @return QString Human-readable rich text (HTML) string.
 */
inline QString StateEnumToString(State state) {
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
