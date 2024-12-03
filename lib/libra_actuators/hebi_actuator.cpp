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
#include "lookup.hpp"  // HEBI

// Project Headers
//   (none)

// Constants

constexpr long kTimeout = 4000;  // ms

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

namespace {  // local to this file

}  // namespace

//------------------------------------------------------------------------------
// !Actuator Commands
//------------------------------------------------------------------------------

bool HebiActuator::Connect() {
    std::cout << "TODO - HebiActuator::Connect()" << std::endl;
    return false;

    // TODO: implementation

    // Create the lookup object
    hebi::Lookup lookup;

    // Wait for the module list to populate, and print out its contents
    std::this_thread::sleep_for(std::chrono::seconds(2));
    {
        std::cout << "Modules found on network (Family|Name):" << std::endl;
        std::shared_ptr<hebi::Lookup::EntryList> entry_list =
            lookup.getEntryList();
        for (auto entry : *entry_list) {
            std::cout << entry.family_ << " | " << entry.name_ << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // Define module names/addresses:
    std::vector<std::string> families;
    families.push_back("CommsTest");

    std::vector<std::string> names;
    names.push_back("One");
    names.push_back("Two");

    std::cout << "Looking up group by name." << std::endl;
    group = lookup.getGroupFromNames(families, names, timeout_ms);
    checkGroup(group ? group->size() : -1);

    std::cout << "Looking up group by family." << std::endl;
    group = lookup.getGroupFromFamily(families[0], timeout_ms);
    checkGroup(group ? group->size() : -1);
}
}  // namespace

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
