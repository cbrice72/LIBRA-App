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
#include <sstream>
#include <thread>

// Other Library Headers
#include "lookup.hpp"  // HEBI

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Actuator Commands
 * !Getters & Setters
 */

/* Constants */

// Improving Readability of Conversions

constexpr double kSecondsPerMin = 60;
constexpr double kRadPerRevolution = 2 * M_PI;

// HEBI Functions

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
      command_(nullptr),
      feedback_(nullptr) {
    // Sanity check inputs since expected use of this object is through an
    // abstract class managed by a non-main thread
    if (debug_mode_) {
        std::string families_str;
        for (const auto& family : families_) {
            families_str += family;
        }
        std::string names_str;
        for (const auto& name : names_) {
            names_str += name;
        }

        std::cout << "[DEBUG] HEBI - Creating actuator with following params:"
                  << "\n  Families: " << families_str
                  << "\n  Names: " << names_str << std::endl;
    }

    // Initialize HEBI objects
    command_ = std::make_unique<hebi::GroupCommand>(names_.size());
    feedback_ = std::make_unique<hebi::GroupFeedback>(names_.size());
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

/**
 * @brief Attempts to establish a connection.
 *
 * @return true if successful, false otherwise
 *
 * @note For a complete example, see HEBI `hebi-cpp-examples` GitHub:
 *       https://github.com/HebiRobotics/hebi-cpp-examples/blob/master/advanced/lookup/lookup_example.cpp
 */
bool HebiActuator::Connect() {
    // Create the lookup object and wait for actuator list to populate
    hebi::Lookup lookup;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Check if any actuators were found
    {
        const auto entry_list = lookup.getEntryList();

        if (entry_list->size() == 0) {
            // Early exit
            std::cout << "[ERROR] HEBI - No actuators found on network!"
                      << std::endl;
            return false;
        }

        if (debug_mode_) {
            // Print out any actuators we found
            std::cout
                << "[DEBUG] HEBI - Found following actuators (Family|Name):\n";

            for (auto entry : *entry_list) {
                std::cout << "  " << entry.family_ << " | " << entry.name_
                          << "\n";
            }
            std::cout << std::endl;
        }
    }

    // Filter lookup for relevant actuator(s)
    group_ = lookup.getGroupFromNames(families_, names_, kLookupTimeout);
    if (group_ == nullptr) {
        std::cout
            << "[ERROR] HEBI - Requested actuator families/names not found!";
        return false;
    }

    // Load safety parameters
    if (!command_->readSafetyParameters("bin/shared/hebi/safety.xml")) {
        std::cout << "[ERROR] HEBI - Failed to load safety parameters!"
                  << std::endl;
        return false;
    }

    // Load gains
    if (!command_->readGains("bin/shared/hebi/gains.xml")) {
        std::cout << "[ERROR] HEBI - Failed to load gain parameters!"
                  << std::endl;
        return false;
    }

    // Initialize actuator(s) with above parameters
    group_->sendCommand(*command_);
    command_->clear();

    // Command actuator(s) to hold current position
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());

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
    if (group_ == nullptr) {
        std::cout << "[ERROR] HEBI - Can't move actuators; not connected!";
        return;
    }

    std::cout << "TODO - HebiActuator::Move()" << std::endl;

    // TODO: implementation
}

/**
 * @brief Sends an actuator stop command.
 */
void HebiActuator::Stop() {
    if (group_ == nullptr) {
        std::cout << "[ERROR] HEBI - Can't stop actuators; not connected!";
        return;
    }

    std::cout << "TODO - HebiActuator::Stop()" << std::endl;

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Returns the current status of the actuator.
 *
 * @return std::string Comma-delimited status messages
 *
 * @note See following HEBI C++ API page for full list of available feedback:
 *       https://files.hebi.us/docs/cpp/cpp-3.11.1/classhebi_1_1GroupFeedback.html
 *
 * @todo Currently, values are only retrieved for the first actuator in the
 *       group. This is fine with LIBRA-II (since only a single HEBI actuator is
 *       used), but for LIBRA-I we need a more programmatic way of sending the data.
 */
std::string HebiActuator::GetStatus() {
    if (group_ == nullptr) {
        return "Not Connected";
    }

    // Get values
    auto target_pos = feedback_->getPositionCommand()[0];  // rad
    auto actual_pos = feedback_->getPosition()[0];         // rad
    auto actual_vel = feedback_->getVelocity()[0];         // rad/s
    auto actual_trq = feedback_->getEffort()[0];           // Nm

    auto defl = feedback_->getDeflection()[0];              // mm?
    auto defl_vel = feedback_->getDeflectionVelocity()[0];  // mms?

    auto volt = feedback_->getVoltage()[0];                  // V
    auto curr = feedback_->getMotorCurrent()[0];             // A
    auto temp = feedback_->getMotorWindingTemperature()[0];  // C
    // alternatively, getBoardTemperature() for electronics

    // Perform conversions
    target_pos *= 180 / M_PI;                          // to deg
    actual_pos *= 180 / M_PI;                          // to deg
    actual_vel *= kSecondsPerMin / kRadPerRevolution;  // to rpm

    std::stringstream status_ss;
    status_ss << "Target Position (deg): " << target_pos
              << "\nActual Position (deg): " << actual_pos
              << "\nActual Velocity (rpm): " << actual_vel
              << "\nActual Torque (Nm): " << actual_trq
              << "\nDeflection (mm): " << defl
              << "\nDeflection Velocity (mm/s): " << defl_vel
              << "\nVoltage (V): " << volt << "\nCurrent (A): " << curr
              << "\nTemperature (C): " << temp;

    return status_ss.str();

    // TODO: if the development environment is ever upgraded to Ubuntu 24.04,
    //       use gcc-13 and set CMAKE_CXX_STANDARD to 20 so we can use <format>
    //       (since Ubuntu 22.04's package provider only goes up to gcc-12)
    /*
    return std::format(
        "Target Position (deg): {};Actual Position (deg): "
        "{};Actual Velocity (rpm): {};Actual Torque (Nm): {};Deflection (mm): "
        "{};Deflection Velocity (mm/s): {};Voltage (V): "
        "{};Current (A): {};Temperature (C): {}"\n
        target_pos, actual_pos, actual_vel, actual_trq, defl, defl_vel, volt,
        curr, temp);
    */
}

/**
 * @brief Returns the target position commanded to the actuator.
 *
 * @return double Target position, in degrees
 */
double HebiActuator::GetTargetPos() {
    if (group_ == nullptr) {
        std::cout << "[ERROR] HEBI - Can't get position; not connected!";
        return 0.0;
    }

    std::cout << "TODO - HebiActuator::GetTargetPos()" << std::endl;
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the current position of the actuator.
 *
 * @return double Actual position, in degrees
 */
double HebiActuator::GetActualPos() {
    if (group_ == nullptr) {
        std::cout << "[ERROR] HEBI - Can't get position; not connected!";
        return 0.0;
    }

    std::cout << "TODO - HebiActuator::GetActualPos()" << std::endl;
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the current effort (torque) of the actuator.
 *
 * @return double Actual torque, in Newtons
 */
double HebiActuator::GetActualTorque() {
    if (group_ == nullptr) {
        std::cout << "[ERROR] HEBI - Can't get torque; not connected!";
        return 0.0;
    }

    std::cout << "TODO - HebiActuator::GetActualTorque()" << std::endl;
    return 0.0;

    // TODO: implementation
}
