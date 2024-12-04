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
    kJointCount  // KEEP THIS LAST!
};

/**
 * @brief Actuator implementations.
 *
 * @note See `abstract_actuator.h`.
 */
enum Type {
    kEpos = 0,  // EPOS4 (Maxon), see `epos_actuator[.h,.cpp]`
    kHebi       // HEBI, see `hebi_actuator[.h,.cpp]`
};

}  // namespace Actuator

/**
 * @brief Parameters necessary for initializing EPOS4 (Maxon) actuators.
 *        One possible variant of `ActuatorDef.params`.
 *
 * @note Access using `std::get<EposParams>()`.
 */
struct EposParams {
    std::string device_name;
    std::string protocol_name;
    std::string interface_name;
    std::string port_name;
    uint baud_rate;
};

/**
 * @brief Parameters necessary for initializing HEBI actuators.
 *        One possible variant of `ActuatorDef.params`.
 *
 * @note Access using `std::get<HebiParams>()`.
 */
struct HebiParams {
    std::vector<std::string> families;
    std::vector<std::string> names;
};

/**
 * @brief Container for defining a LIBRA actuator.
 *
 * @note Initialization parameters (`params`) are implementation-defined, so we
 *       use a std::variant as a union.
 * @note See use in `ArmThread::ArmThread()`.
 */
struct ActuatorDef {
    Actuator::Joint joint;                        // joint number
    Actuator::Type type;                          // actuator make
    std::variant<EposParams, HebiParams> params;  // union for parameters
};
