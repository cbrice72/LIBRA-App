/******************************************************************************
 * @file   actuator_defs.h
 * @brief  Type definitions for initializing LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

namespace Actuator {  // place enums in this namespace to improve readability

/**
 * @brief LIBRA-II joint names.
 */
static enum Joint {
    kYaw = 0,
    kPitch,
    kJointCount  // KEEP THIS LAST!
};

/**
 * @brief Actuator implementations.
 *
 * @note See `abstract_actuator.h`.
 */
static enum Type {
    kEpos = 0,  // EPOS4 (Maxon), see `epos_actuator[.h,.cpp]`
    kHebi       // HEBI, see `hebi_actuator[.h,.cpp]`
};

}  // namespace Actuator

/**
 * @brief Container for defining a LIBRA actuator.
 *
 * @note Initialization parameters (`params`) are implementation-dependent.
 * @note See `ArmThread::ArmThread()`.
 */
struct ActuatorDef {
    Actuator::Joint joint;            // joint number
    Actuator::Type type;              // actuator make
    std::vector<std::string> params;  // initialization parameters
}
