/******************************************************************************
 * @file   joint_defs.h
 * @brief  Common definitions for LIBRA joints; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstdint>
#include <iostream>

// Other Library Headers
#include "actuator_defs.h"

/* --- TABLE OF CONTENTS ---
 * !Constants
 * !Conversion Functions
 * !Helper Functions
 */

/**
 * @brief Common definitions for controlling LIBRA joints. Not to be confused
 *        with "actuator", which refers to the physical thing.
 */
namespace Joint {

//------------------------------------------------------------------------------
// !Constants
//------------------------------------------------------------------------------

/**
 * @brief Joint direction (revolute only).
 *
 * @see TODO
 */
enum class Type : uint8_t { kRoll = 0, kPitch, kYaw };

/**
 * @brief Joint names.
 *
 * @note This is what is both displayed to the user and published to the ROS2
 *       ecosystem, not `Actuator::Name`, which is for the physical hardware.
 *
 * @note HEBI actuator names MUST start at 0 for everything to line up properly,
 *       thanks to the design of `Actuator::GetHebiDefault()`. I know this
 *       confusing (it has already led to a frustrating bug hunt), but I have
 *       more important things to do right now than refactor this...
 *
 * @see Actuator::Name
 */
enum Name : uint8_t {
#if LIBRA_VERSION == 1
    kRoll = 0,
    kPitch,
    kJ1,  // Yaw
    kJ2,  // Yaw
    kJ3,  // Pitch
#elif LIBRA_VERSION == 2
    kPitch = 0,
    kYaw,
#endif
    kUndefined  // keep this last!
};

//------------------------------------------------------------------------------
// !Conversion Functions
//------------------------------------------------------------------------------

/**
 * @brief Converts joint `Name` enum value to its corresponding string name.
 *
 * @param name Named enum value of the joint
 * @return std::string String name of the corresponding joint
 */
inline std::string NameEnumToString(Name name) {
    switch (name) {
#if LIBRA_VERSION == 1
        case Name::kRoll:
            return "Roll";
        case Name::kPitch:
            return "Pitch";
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
 * @brief Converts a 1-DoF actuator's real position to its joint's effective
 *        position.
 *
 * @note Positive X = forward, positive Y = leftward, and positive Z = upward.
 *       By extension, positive roll = leftward, positive pitch = upward, and
 *       positive yaw = leftward.
 *
 * @param joint Which joint to calculate for
 * @param val The value to convert
 * @return double Effective position of the joint
 *
 * @see Actuator::JointToActuatorSimple HebiThread::PublishState
 */
inline double ActuatorToJointSimple(Name joint, double val) {
    switch (joint) {
#if LIBRA_VERSION == 1
        case Name::kRoll:
        case Name::kPitch:
            std::cerr
                << "[ERROR] ActuatorToJointSimple: This joint is part of a "
                   "complex system! Please use ActuatorToJointDifferential."
                << std::endl;
            return 0;
        case Name::kJ1:
            return val;
        case Name::kJ2:
            return -val;
        case Name::kJ3:
            return -val;
#elif LIBRA_VERSION == 2
        case Name::kPitch:
            return -val;
        case Name::kYaw:
            return -val;
#endif
        default:
            std::cerr << "[ERROR] ActuatorToJointSimple: Unknown joint!"
                      << std::endl;
            return 0;
    }
}

/**
 * @brief Derives the effective position of a differential joint based on its
 *        linked actuators' real positions.
 *
 * @param joint Which joint to calculate for
 * @param ma Position of actuator "MA" (left side of 2-DoF Joint)
 * @param mb Position of actuator "MB" (right side of 2-DoF Joint)
 * @return double Effective position of the joint
 *
 * @see Actuator::JointToActuatorDifferential HebiThread::PublishState
 */
inline double ActuatorToJointDifferential(Name joint, double ma, double mb) {
    switch (joint) {
#if LIBRA_VERSION == 1
        case Name::kRoll:
            return -(ma + mb) / 2.0;  // positive roll = leftward
        case Name::kPitch:
            return (ma - mb) / 2.0;  // positive pitch = upward
#endif
        default:
            std::cerr << "[ERROR] ActuatorToJointDifferential: Provided joint "
                         "name is not part of a known complex system!"
                      << std::endl;
            return 0;
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Returns an ordered list of joints controlled by HEBI actuators.
 *
 * @note No logic; return value determined solely by `LIBRA_VERSION`.
 *
 * @return std::vector<std::string> Ordered list of string names
 */
inline std::vector<std::string> GetHebiStrings() {
#if LIBRA_VERSION == 1
    return {"Roll", "Pitch", "J1", "J2", "J3"};
#elif LIBRA_VERSION == 2
    return {"Pitch"};
#endif
    return {"Undefined"};
}

}  // namespace Joint
