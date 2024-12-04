/******************************************************************************
 * @file   epos_actuator.cpp
 * @brief  Control class for EPOS (Maxon) actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "epos_actuator.h"

// C++ Standard Library Headers
#include <iostream>

// Other Library Headers
//   (none)

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Actuator Commands
 * !Getters & Setters
 */

/**
 * @brief Standard constructor.
 *
 */
EposActuator::EposActuator(std::string device_name, std::string protocol_name,
                           std::string interface_name, std::string port_name,
                           uint baud_rate, const bool& debug_mode)
    : AbstractActuator(debug_mode),
      device_name_(std::move(device_name)),
      protocol_name_(std::move(protocol_name)),
      interface_name_(std::move(interface_name)),
      port_name_(std::move(port_name)),
      baud_rate_(baud_rate) {}

/**
 * @brief Standard destructor.
 *
 */
EposActuator::~EposActuator() {
    std::cout << "TODO - EposActuator::~EposActuator()" << std::endl;

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {  // local to this file

}  // namespace

//------------------------------------------------------------------------------
// !Actuator Commands
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish a connection.
 *
 * @return true if successful, false otherwise
 */
bool EposActuator::Connect() {
    std::cout << "TODO - EposActuator::Connect()" << std::endl;
    return false;

    // TODO: implementation
}

/**
 * @brief Terminates the active connection.
 *
 * @return true if successful, false otherwise
 */
bool EposActuator::Disconnect() {
    std::cout << "TODO - EposActuator::Disconnect()" << std::endl;
    return false;

    // TODO: implementation
}

/**
 * @brief Sends an actuator movement command.
 *
 * @param deg Target angle (absolute)
 *
 * @note If this actuator doesn't directly accept degrees as part of its
 *       movement command, the translation should be done within this function.
 */
void EposActuator::Move(double deg) {
    std::cout << "TODO - EposActuator::Move()" << std::endl;

    // TODO: implementation
}

/**
 * @brief Sends an actuator stop command.
 */
void EposActuator::Stop() {
    std::cout << "TODO - EposActuator::Stop()" << std::endl;

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

std::string EposActuator::GetStatus() {
    std::cout << "TODO - EposActuator::GetStatus()" << std::endl;
    return "";

    // TODO: implementation
}

double EposActuator::GetTargetPos() {
    std::cout << "TODO - EposActuator::GetTargetPos()" << std::endl;
    return 0.0;

    // TODO: implementation
}

double EposActuator::GetActualPos() {
    std::cout << "TODO - EposActuator::GetActualPos()" << std::endl;
    return 0.0;

    // TODO: implementation
}

double EposActuator::GetActualTorque() {
    std::cout << "TODO - EposActuator::GetActualTorque()" << std::endl;
    return 0.0;

    // TODO: implementation
}
