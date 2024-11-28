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
EposActuator::EposActuator() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

/**
 * @brief Standard destructor.
 *
 */
EposActuator::~EposActuator() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Actuator Commands
//------------------------------------------------------------------------------

bool EposActuator::Connect() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

bool EposActuator::Disconnect() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

void EposActuator::Move(double deg) {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

void EposActuator::Stop() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

std::string EposActuator::GetStatus() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

double EposActuator::GetTargetPos() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

double EposActuator::GetActualPos() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}

double EposActuator::GetActualTorque() {
    std::cout << "TODO" << std::endl;

    // TODO: implementation
}
