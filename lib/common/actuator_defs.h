/******************************************************************************
 * @file   actuator_defs.h
 * @brief  Common definitions for physical LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
#include <vector>

// Other Library Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Constants
 * !Conversion Functions
 * !Helper Functions
 */

/**
 * @brief Common definitions for controlling LIBRA actuators.
 */
namespace Actuator {

//------------------------------------------------------------------------------
// !Constants
//------------------------------------------------------------------------------

// Total count of each actuator type
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
 * @brief Actuator implementations.
 *
 * @see AbstractActuatorThread MainWindow::HandleActuatorStatus
 */
enum Type {
    kHebi = 0,  // HEBI
#if LIBRA_VERSION == 2
    kEpos  // EPOS (Maxon)
#endif
};

/**
 * @brief Canonical actuator names.
 *
 * @note HEBI actuator names MUST start at 0 for `GetDefaultHebiActuatorList()`
 *       to work properly. I know this is terrible design, but I have more
 *       important things to do right now than refactor this...
 *
 * @see GetDefaultHebiActuatorList HebiThread::run
 */
enum Name {
#if LIBRA_VERSION == 1
    kMA = 0,  // HEBI
    kMB,      // HEBI
    kJ1,      // HEBI
    kJ2,      // HEBI
    kJ3,      // HEBI
#elif LIBRA_VERSION == 2
    kPitch = 0,  // HEBI
    kYaw,        // EPOS
#endif
    kUndefined  // keep this last!
};

/**
 * @brief Primary actuator feedback types.
 *
 * @note Secondary feedback should go in each actuator implementation's
 *       `ReportStatus()` override.
 *
 * @see AbstractActuatorThread::ReportStatus
 */
enum Feedback { kTargetPos = 0, kActualPos, kActualTorque };

//------------------------------------------------------------------------------
// !Conversion Functions
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
 */
inline std::string NameEnumToString(Name name) {
    switch (name) {
#if LIBRA_VERSION == 1
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
#elif LIBRA_VERSION == 2
        case Name::kYaw:
            return "Yaw";
        case Name::kPitch:
            return "Pitch";
#endif
        default:
            return "Undefined";
    }
}

/**
 * @brief Converts actuator `Type` enum value to its corresponding string name.
 *
 * @param name Actuator implementation enum value
 * @return std::string String name of the corresponding actuator type
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

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Returns the default HEBI actuator names based on the `LIBRA_VERSION`
 *        the app is built with.
 *
 * @return std::vector<std::string> Ordered list of HEBI actuators
 */
inline std::vector<std::string> GetDefaultHebiActuatorList() {
    std::vector<std::string> names;

    names.reserve(Actuator::kJointCountHebi);  // for efficiency
    for (int i = 0; i < Actuator::kJointCountHebi; ++i) {
        names.push_back(NameEnumToString(static_cast<Name>(i)));
    }

    return names;
}

}  // namespace Actuator
