/******************************************************************************
 * @file   hebi_actuator.cpp
 * @brief  Control class for LIBRA HEBI actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "hebi_actuator.h"

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
 */
HebiActuator::HebiActuator(bool debug_mode) : AbstractActuator(debug_mode) {
    std::cout << "TODO - HebiActuator::HebiActuator()" << std::endl;

    // TODO: implementation
}

/**
 * @brief Standard destructor.
 */
HebiActuator::~HebiActuator() {
    std::cout << "TODO - HebiActuator::~HebiActuator()" << std::endl;

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Actuator Commands
//------------------------------------------------------------------------------

bool HebiActuator::Connect() {
    std::cout << "TODO - HebiActuator::Connect()" << std::endl;
    return false;

    // TODO: implementation
}

bool HebiActuator::Disconnect() {
    std::cout << "TODO - HebiActuator::Disconnect()" << std::endl;
    return false;

    // TODO: implementation
}

void HebiActuator::Move(double deg) {
    std::cout << "TODO - HebiActuator::Move()" << std::endl;

    // TODO: implementation
}

void HebiActuator::Stop() {
    std::cout << "TODO - HebiActuator::Stop()" << std::endl;

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

std::string HebiActuator::GetStatus() {
    std::cout << "TODO - HebiActuator::GetStatus()" << std::endl;
    return "";

    // TODO: implementation
}

double HebiActuator::GetTargetPos() {
    std::cout << "TODO - HebiActuator::GetTargetPos()" << std::endl;
    return 0.0;

    // TODO: implementation
}

double HebiActuator::GetActualPos() {
    std::cout << "TODO - HebiActuator::GetActualPos()" << std::endl;
    return 0.0;

    // TODO: implementation
}

double HebiActuator::GetActualTorque() {
    std::cout << "TODO - HebiActuator::GetActualTorque()" << std::endl;
    return 0.0;

    // TODO: implementation
}
