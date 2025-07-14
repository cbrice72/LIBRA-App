/******************************************************************************
 * @file   actuator_defs.h
 * @brief  Common definitions for LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
#include <variant>
#include <vector>

// Other Library Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Constants
 * !Helper Functions
 */

/**
 * @brief Common definitions for controlling LIBRA actuators.
 */
namespace Actuator {

//------------------------------------------------------------------------------
// !Constants
//------------------------------------------------------------------------------

#if LIBRA_VERSION == 1
constexpr int kJointCountHebi = 5;
constexpr int kJointCountEpos = 0;
#elif LIBRA_VERSION == 2
constexpr int kJointCountHebi = 1;
constexpr int kJointCountEpos = 1;
#else
constexpr int kJointCountHebi = 0;
constexpr int kJointCountEpos = 0;
#endif

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
    kUndefined  // KEEP THIS LAST (used in StringToNameEnum)
};

/**
 * @brief Actuator implementations.
 *
 * @see abstract_actuator_thread
 */
enum Type {
    kHebi = 0,  // HEBI
#if LIBRA_VERSION == 2
    kEpos  // EPOS4 (Maxon)
#endif
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
 * @param name_str Name of the actuator
 * @return Actuator::Name Named enum value of the corresponding actuator
 */
inline Name StringToNameEnum(const std::string& name_str) {
#if LIBRA_VERSION == 1
    if (name_str == "MA") {
        return Name::kMA;
    } else if (name_str == "MB") {
        return Name::kMB;
    } else if (name_str == "J1") {
        return Name::kJ1;
    } else if (name_str == "J2") {
        return Name::kJ2;
    } else if (name_str == "J3") {
        return Name::kJ3;
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
 * @param name Named enum value of the actuator
 * @return std::string String name of the corresponding actuator
 *
 * @note Only really useful for debugging.
 */
inline std::string NameEnumToString(Name name) {
#if LIBRA_VERSION == 1
    switch (name) {
        case Name::kMA:
            return "MA";
        case Name::kMB:
            return "MB";
        case Name::kJ1:
            return "J1";
        case Name::kJ2:
            return "J2";
        case Name::kJ3:
            return "J3";
        default:
            return "Undefined";
    }
#elif LIBRA_VERSION == 2
    switch (name) {
        case Name::kYaw:
            return "Yaw";
        case Name::kPitch:
            return "Pitch";
        default:
            return "Undefined";
    }
#endif
}

/**
 * @brief Converts actuator `Type` enum value to its corresponding string name.
 *
 * @param name Actuator implementation enum value
 * @return std::string String name of the corresponding actuator type
 *
 * @note Only really useful for debugging.
 */
inline std::string TypeEnumToString(Type type) {
    switch (type) {
        case Type::kHebi:
            return "HEBI";
#if LIBRA_VERSION == 2
        case Type::kEpos:
            return "EPOS";
#endif
        default:
            return "Undefined";
    }
}

/**
 * @brief Converts actuator `Feedback` enum value to its corresponding string name.
 *
 * @param feedback Feedback type
 * @return std::string String feedback type
 *
 * @note Only really useful for debugging.
 */
inline std::string FeedbackEnumToString(Feedback feedback) {
    switch (feedback) {
        case Feedback::kTargetPos:
            return "TargetPos";
        case Feedback::kActualPos:
            return "ActualPos";
        case Feedback::kActualTorque:
            return "ActualTorque";
        default:
            return "Undefined";
    }
}

}  // namespace Actuator
