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
 * @see Actuator::Name
 */
enum class Name : uint8_t {
#if LIBRA_VERSION == 1
    kRoll = 0,
    kPitch,
    kJ1,  // Yaw
    kJ2,  // Yaw
    kJ3,  // Pitch
#elif LIBRA_VERSION == 2
    kYaw = 0,
    kPitch,
#endif
    kUndefined  // keep this last!
};

//------------------------------------------------------------------------------
// !Conversion Functions
//------------------------------------------------------------------------------

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
 * @see HebiThread::PublishState
 */
inline double ActuatorToJointSimple(Joint::Name joint, double val) {
    switch (joint) {
#if LIBRA_VERSION == 1
        case Joint::Name::kRoll:
        case Joint::Name::kPitch:
            std::cerr
                << "[ERROR] ActuatorToJointSimple: This joint is part of a "
                   "complex system! Please use ActuatorToJointDifferential."
                << std::endl;
            return 0;
        case Actuator::Name::kJ1:
            return val;
        case Actuator::Name::kJ2:
            return -val;
        case Actuator::Name::kJ3:
            return -val;
#elif LIBRA_VERSION == 2
        case Actuator::Name::kPitch:
            return val;  // TODO: verify this
        case Actuator::Name::kYaw:
            return val;  // TODO: verify this
#endif
        default:
            std::cerr << "[ERROR] ActuatorToJointSimple: Unknown joint!"
                      << std::endl;
            return 0;
    }
}

/**
 * @brief Converts a simple joint's effective position to its corresponding
 *        actuator's real position.
 *
 * @param actuator Which actuator to calculate for
 * @param val The value to convert
 * @return double Real position of the actuator
 *
 * @note Currently unused; provided for completeness
 */
inline double JointToActuatorSimple(Actuator::Name actuator, double val) {
    switch (actuator) {
#if LIBRA_VERSION == 1
        case Actuator::Name::kMA:
        case Actuator::Name::kMB:
            std::cerr
                << "[ERROR] JointToActuatorSimple: This actuator is part of a "
                   "complex system! Please use JointToActuatorDifferential."
                << std::endl;
            return 0;
        case Actuator::Name::kJ1:
            return val;
        case Actuator::Name::kJ2:
            return -val;
        case Actuator::Name::kJ3:
            return -val;
#elif LIBRA_VERSION == 2
        case Actuator::Name::kPitch:
            return val;  // TODO: verify this
        case Actuator::Name::kYaw:
            return val;  // TODO: verify this
#endif
        default:
            std::cerr << "[ERROR] JointToActuatorSimple: Unknown actuator!"
                      << std::endl;
            return 0;
    }
}

#if LIBRA_VERSION == 1
/**
 * @brief Derives the effective position of a differential joint based on its
 *        linked actuators' real positions.
 *
 * @param joint Which joint to calculate for
 * @param ma Position of actuator "MA" (left side of 2-DoF Joint)
 * @param mb Position of actuator "MB" (right side of 2-DoF Joint)
 * @return double Effective position of the joint
 *
 * @see HebiThread::PublishState
 */
inline double ActuatorToJointDifferential(Joint::Name joint, double ma,
                                          double mb) {
    switch (joint) {
        case Joint::Name::kRoll:
            return -(ma + mb) / 2.0;
        case Joint::Name::kPitch:
            return -(ma - mb) / 2.0;
        default:
            std::cerr << "[ERROR] ActuatorToJointDifferential: Provided joint "
                         "name is not part of a known complex system!"
                      << std::endl;
            return 0;
    }
}

/**
 * @brief Derives the real position of a linked actuator based on its
 *        differential joints' effective positions.
 *
 * @param actuator Which actuator to calculate for
 * @param roll Position of central roll joint
 * @param pitch Position of central pitch joint
 * @return double Real position of the actuator
 *
 * @note Currently unused; provided for completeness
 */
inline double JointToActuatorDifferential(Actuator::Name actuator, double roll,
                                          double pitch) {
    switch (actuator) {
        case Actuator::Name::kMA:
            return -roll - pitch;
        case Actuator::Name::kMB:
            return -roll + pitch;
        default:
            std::cerr
                << "[ERROR] JointToActuatorDifferential: Provided actuator "
                   "name is not part of a known complex system!"
                << std::endl;
            return 0;
    }
}
#endif

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
