/******************************************************************************
 * @file   actuator_defs.h
 * @brief  Type definitions for initializing LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
#include <variant>
#include <vector>

/* --- TABLE OF CONTENTS ---
 * !Constants
 * !Helper Functions
 */

namespace Actuator {  // class not necessary, but namespace improves readability

//------------------------------------------------------------------------------
// !Constants
//------------------------------------------------------------------------------

/**
 * @brief LIBRA actuator names.
 */
enum Name {
#if LIBRA_VERSION == 1
    kMA = 0,
    kMB,
    kJ1,
    kJ2,
    kJ3,
#elif LIBRA_VERSION == 2
    kYaw = 0,
    kPitch,
#endif
    kJointCount,  // included for convenience
    kUndefined    // KEEP THIS LAST
};

/**
 * @brief Actuator implementations.
 *
 * @see abstract_actuator_thread
 */
enum Type {
    kEpos = 0,  // EPOS4 (Maxon)
    kHebi       // HEBI
};

/**
 * @brief Important actuator feedback types.
 *
 * @note Secondary feedback should go in each actuator implementation's
 *       `ReportStatus()` override.
 *
 * @see AbstractActuatorThread::ReportStatus
 */
enum Feedback { kTargetPos = 0, kActualPos, kActualTorque };

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Converts actuator string name to its corresponding `Name` enum value.
 *
 * @param name_str Name of the actuator.
 * @return Actuator::Name Named enum value of the corresponding actuator.
 */
inline Name StringToNameEnum(const std::string& name_str) {
#if LIBRA_VERSION == 1
    if (name_str == "MA") {
        return kMA;
    } else if (name_str == "MB") {
        return kMB;
    } else if (name_str == "J1") {
        return kJ1;
    } else if (name_str == "J2") {
        return kJ2;
    } else if (name_str == "J3") {
        return kJ3;
    }
#elif LIBRA_VERSION == 2
    if (name_str == "Yaw") {
        return kYaw;
    } else if (name_str == "Pitch") {
        return kPitch;
    }
#endif
    return kUndefined;
}

/**
 * @brief Converts actuator `Name` enum value to its corresponding string name.
 *
 * @param name Named enum value of the actuator.
 * @return std::string String name of the corresponding actuator.
 *
 * @note Only really useful for debugging.
 */
inline std::string NameEnumToString(Name name) {
#if LIBRA_VERSION == 1
    switch (name) {
        case kMA:
            return "MA";
        case kMB:
            return "MB";
        case kJ1:
            return "J1";
        case kJ2:
            return "J2";
        case kJ3:
            return "J3";
        default:
            return "Undefined";
    }
#elif LIBRA_VERSION == 2
    switch (name) {
        case kYaw:
            return "Yaw";
        case kPitch:
            return "Pitch";
        default:
            return "Undefined";
    }
#endif
}

}  // namespace Actuator
