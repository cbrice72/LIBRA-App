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

namespace Actuator {  // place enums in this namespace to improve readability

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
    kUndefined  // KEEP THIS LAST
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

}  // namespace Actuator
