/******************************************************************************
 * @file   arduino_defs.h
 * @brief  Common definitions for devices managed by LIBRA arduinos; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Other Library Headers
#include <QString>  // Qt::Core

/* --- TABLE OF CONTENTS ---
 * !Manip Constants
 * !Manip Helpers
 * !Water Constants
 * !Water Helpers
 */

namespace Manip {  // class not necessary, but namespace improves readability

//------------------------------------------------------------------------------
// !Manip Constants
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Manip Helpers
//------------------------------------------------------------------------------

}  // namespace Manip

namespace Water {  // class not necessary, but namespace improves readability

//------------------------------------------------------------------------------
// !Water Constants
//------------------------------------------------------------------------------

/**
 * @brief Fluid system side (see individual constants for description).
 */
enum class Side {
    kA = 0  // LIBRA-I: left, LIBRA-II: only one
#if LIBRA_VERSION == 1
    ,
    kB  // LIBRA-I: right
#endif
};

/**
 * @brief Fluid system state.
 */
enum class State { kStopped = 0, kFilling, kDraining };

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
#if LIBRA_VERSION == 1
        case Side::kB:
            return "B";
#endif
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
