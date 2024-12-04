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
      baud_rate_(baud_rate) {
    if (debug_mode_) {
        std::cout << "[DEBUG] Creating EPOS actuator with following params:"
                  << "\n  Device: " << device_name_
                  << "\n  Protocol: " << protocol_name_
                  << "\n  Interface: " << interface_name_
                  << "\n  Port: " << port_name_
                  << "\n  Baud Rate: " << baud_rate_ << std::endl;
    }
}

/**
 * @brief Standard destructor.
 *
 */
EposActuator::~EposActuator() {
    std::cout << "TODO - EposActuator::~EposActuator()\n";

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
    std::cout << "TODO - EposActuator::Connect()\n";
    return false;

    // TODO: implementation
}

/**
 * @brief Terminates the active connection.
 *
 * @return true if successful, false otherwise
 */
bool EposActuator::Disconnect() {
    std::cout << "TODO - EposActuator::Disconnect()\n";
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
    std::cout << "TODO - EposActuator::Move()\n";

    // TODO: implementation
}

/**
 * @brief Sends an actuator stop command.
 */
void EposActuator::Stop() {
    std::cout << "TODO - EposActuator::Stop()\n";

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Returns the current status of the actuator.
 *
 * @return std::string Semicolon-delimited status messages
 */
std::string EposActuator::GetStatus() {
    std::cout << "TODO - EposActuator::GetStatus()\n";
    return "";

    // TODO: implementation
}

/**
 * @brief Returns the target position commanded to the actuator.
 *
 * @return double Target position, in degrees
 */
double EposActuator::GetTargetPos() {
    std::cout << "TODO - EposActuator::GetTargetPos()\n";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the current position of the actuator.
 *
 * @return double Actual position, in degrees
 */
double EposActuator::GetActualPos() {
    std::cout << "TODO - EposActuator::GetActualPos()\n";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the current effort (torque) of the actuator.
 *
 * @return double Actual torque, in Newtons
 */
double EposActuator::GetActualTorque() {
    std::cout << "TODO - EposActuator::GetActualTorque()\n";
    return 0.0;

    // TODO: implementation
}
