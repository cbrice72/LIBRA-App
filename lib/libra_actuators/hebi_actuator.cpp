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
#include <thread>

// Other Library Headers
#include "group_feedback.hpp"  // HEBI
#include "lookup.hpp"          // HEBI

// Project Headers
//   (none)

// Constants

constexpr long kTimeout = 4000;  // ms

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Actuator Commands
 * !Getters & Setters
 */

/* Constants */
constexpr int32_t kLookupTimeout = 4000;  // ms

/**
 * @brief Standard constructor.
 *
 * @param families Families of actuators to search for names in
 * @param names Names of actuators to connect to
 * @param debug_mode Whether verbose debug text should be output
 */
HebiActuator::HebiActuator(std::vector<std::string> families,
                           std::vector<std::string> names,
                           const bool& debug_mode)
    : AbstractActuator(debug_mode),
      families_(std::move(families)),
      names_(std::move(names)),
      group_(nullptr),
      current_pos_{0.0, 0.0, 0.0},
      current_vel_{0.0, 0.0, 0.0},
      current_trq_{0.0, 0.0, 0.0},
      current_deflection_{0.0, 0.0, 0.0},
      current_voltage_{0.0, 0.0, 0.0},
      current_current_{0.0, 0.0, 0.0},
      current_temp_{0.0, 0.0, 0.0} {
    if (debug_mode_) {
        std::string families_str;
        for (const auto& family : families_) {
            families_str += family;
        }
        std::string names_str;
        for (const auto& name : names_) {
            names_str += name;
        }

        std::cout << "[DEBUG] Creating HEBI actuator with following params:"
                  << "\n  Families: " << families_str
                  << "\n  Names: " << names_str << std::endl;
    }
}

/**
 * @brief Standard destructor.
 */
HebiActuator::~HebiActuator() {
    std::cout << "TODO - HebiActuator::~HebiActuator()\n";

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
 *
 * @note For a complete example, see HEBI `hebi-cpp-examples` GitHub:
 *       https://github.com/HebiRobotics/hebi-cpp-examples/blob/master/advanced/lookup/lookup_example.cpp
 */
bool HebiActuator::Connect() {
    // Create the lookup object and wait for module list to populate
    hebi::Lookup lookup;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Check if any modules were found
    {
        const auto entry_list = lookup.getEntryList();

        if (entry_list->size() == 0) {
            // Early exit
            std::cout << "[ERROR] No HEBI actuators found on network!\n";
            return false;
        }

        if (debug_mode_) {
            // Print out any modules we found
            std::cout
                << "[DEBUG] HEBI modules found on network (Family|Name):\n";

            for (auto entry : *entry_list) {
                std::cout << "  " << entry.family_ << " | " << entry.name_
                          << "\n";
            }
            std::cout << std::endl;
        }
    }

    // Filter lookup for relevant module(s)
    group_ = lookup.getGroupFromNames(families_, names_, kLookupTimeout);
    if (!group_) {
        std::cout
            << "[ERROR] Given actuator families/names not found on network!";
        return false;
    }

    // Add a callback to save feedback in a background thread
    // NOTE: This C++11 "lambda function" allows us to locally define a function
    //       in the addFeedbackHandler() parameter list. For more information,
    //       see: https://en.cppreference.com/w/cpp/language/lambda
    group_->addFeedbackHandler([this](const hebi::GroupFeedback& feedback) {
        auto pos = feedback.getPosition();
        current_pos_ = {pos(0, 0), pos(0, 1), pos(0, 2)};
        auto vel = feedback.getVelocity();
        current_vel_ = {vel(0, 0), vel(0, 1), vel(0, 2)};
        auto trq = feedback.getEffort();
        current_trq_ = {trq(0, 0), trq(0, 1), trq(0, 2)};
        auto defl = feedback.getDeflection();
        current_deflection_ = {defl(0, 0), defl(0, 1), defl(0, 2)};

        auto volt = feedback.getVoltage();
        current_voltage_ = {volt(0, 0), volt(0, 1), volt(0, 2)};
        auto curr = feedback.getMotorCurrent();
        current_current_ = {curr(0, 0), curr(0, 1), curr(0, 2)};
        auto temp = feedback.getMotorHousingTemperature();
        current_temp_ = {temp(0, 0), temp(0, 1), temp(0, 2)};
    });

    return true;
}

/**
 * @brief Terminates the active connection.
 *
 * @return true if successful, false otherwise
 */
bool HebiActuator::Disconnect() {
    if (group_) {
        // Destructing hebi::Group automatically cleans it up
        group_.reset();
    }

    return true;
}

/**
 * @brief Sends an actuator movement command.
 *
 * @param deg Target angle (absolute)
 *
 * @note If this actuator doesn't directly accept degrees as part of its
 *       movement command, the translation should be done within this function.
 */
void HebiActuator::Move(double deg) {
    std::cout << "TODO - HebiActuator::Move()\n";

    // TODO: implementation
}

/**
 * @brief Sends an actuator stop command.
 */
void HebiActuator::Stop() {
    std::cout << "TODO - HebiActuator::Stop()\n";

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
std::string HebiActuator::GetStatus() {
    std::cout << "TODO - HebiActuator::GetStatus()\n";
    return "";

    // TODO: implementation
}

/**
 * @brief Returns the target position commanded to the actuator.
 *
 * @return double Target position, in degrees
 */
double HebiActuator::GetTargetPos() {
    std::cout << "TODO - HebiActuator::GetTargetPos()\n";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the current position of the actuator.
 *
 * @return double Actual position, in degrees
 */
double HebiActuator::GetActualPos() {
    std::cout << "TODO - HebiActuator::GetActualPos()\n";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the current effort (torque) of the actuator.
 *
 * @return double Actual torque, in Newtons
 */
double HebiActuator::GetActualTorque() {
    std::cout << "TODO - HebiActuator::GetActualTorque()\n";
    return 0.0;

    // TODO: implementation
}
