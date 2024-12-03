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
EposActuator::EposActuator(bool debug_mode) : AbstractActuator(debug_mode) {
    std::cout << "TODO - EposActuator::EposActuator()" << std::endl;

    // TODO: implementation
}

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

bool EposActuator::Connect() {
    std::cout << "TODO - EposActuator::Connect()" << std::endl;
    return false;

    // TODO: implementation
}

bool EposActuator::Disconnect() {
    std::cout << "TODO - EposActuator::Disconnect()" << std::endl;
    return false;

    // TODO: implementation
}

void EposActuator::Move(double deg) {
    std::cout << "TODO - EposActuator::Move()" << std::endl;

    // TODO: implementation
}

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
