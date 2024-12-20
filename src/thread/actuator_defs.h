/******************************************************************************
 * @file   actuator_defs.h
 * @brief  Type definitions for initializing LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

#include <string>
#include <variant>
#include <vector>

namespace Actuator {  // place enums in this namespace to improve readability

/**
 * @brief LIBRA-II joint names.
 */
enum Joint {
    kYaw = 0,
    kPitch,
    kJointCount,  // KEEP THIS LAST!
    kUndefined
};

/**
 * @brief Actuator implementations.
 *
 * @see abstract_actuator_thread.h
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
 * @see AbstractActuatorThread::ReportStatus()
 */
enum Feedback { kTargetPos = 0, kActualPos, kActualTorque };

}  // namespace Actuator
