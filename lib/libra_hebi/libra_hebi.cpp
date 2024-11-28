/******************************************************************************
 * @file   libra_hebi.cpp
 * @brief  Control code for LIBRA-II arm HEBI actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra_hebi.h"

// C++ Standard Library Headers
#include <chrono>
#include <iostream>

// Other Library Headers
//   (none)

// Project Headers
//   (none)

constexpr uint8_t kNodeCount = 5;  // total number of HEBI actuators

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Actuator Commands
 * !Getters & Setters
 */

/**
 * @brief Standard constructor.
 */
LibraHebi::LibraHebi() : start_time_(GetCurrentTimeInSec()) {
    command_ = std::make_unique<hebi::GroupCommand>(kNodeCount);
    feedback_ = std::make_unique<hebi::GroupFeedback>(kNodeCount);

    // Timer interrupt: process interrupts every 10 ms
    /* TODO(brice.c.aa)
    timeSetEvent(10, 0, Callback, reinterpret_cast<DWORD>(this),
                 TIME_PERIODIC | TIME_CALLBACK_FUNCTION);
    */
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Retrieves the current system time (i.e., time since epoch) in seconds.
 *
 * @return Current system time in seconds
 */
std::chrono::system_clock::rep LibraHebi::GetCurrentTimeInSec() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(now).count();
}

/**
 * @brief TODO.
 */
void LibraHebi::Loop() {
    command_->setVelocity(Eigen::VectorXd::Zero(kNodeCount));

    if (trajectory_ != nullptr) {
        const double time = (GetCurrentTimeInSec() - start_time_) / 1000.0;
        if (trajectory_->getDuration() > time) {
            Eigen::VectorXd pos_cmd(kNodeCount);
            Eigen::VectorXd vel_cmd(kNodeCount);
            trajectory_->getState(time, &pos_cmd, &vel_cmd, nullptr);
            command_->setPosition(pos_cmd);
            command_->setVelocity(vel_cmd);
        }
    }

    if (group_ != nullptr) {
        group_->sendCommand(*command_);
        group_->getNextFeedback(*feedback_);
    }
}

//------------------------------------------------------------------------------
// !Actuator Commands
//------------------------------------------------------------------------------

/**
 * @brief Connects to the HEBI actuator group and sets their parameters.
 *
 * @return true if connection and initialization succeeded; false otherwise
 */
bool LibraHebi::Connect() {
    hebi::Lookup lookup;
    group_ = lookup.getGroupFromNames({"X8-16"}, {"Pitch"});
    if (group_ == nullptr) {
        std::cerr << "[ERROR] HEBI - Failed to connect to actuators!\n";
        command_->setPosition(Eigen::VectorXd::Zero(kNodeCount));
        return false;
    }

    if (!command_->readSafetyParameters("shared/hebi/safety.xml")) {
        std::cerr << "[ERROR] HEBI - Failed to load safety parameters!\n";
        return false;
    }

    if (!command_->readGains("shared/hebi/gain.xml")) {
        std::cerr << "[ERROR] HEBI - Failed to load gain parameters!\n";
        return false;
    }

    group_->sendCommand(*command_);  // initialize the actuator group
    command_->clear();
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());  // hold current position

    return true;
}

/**
 * @brief TODO.
 *
 * @param pitch Desired pitch angle, in degrees
 */
void LibraHebi::Move(double pitch) {
    Eigen::MatrixXd positions(kNodeCount, 2);
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(kNodeCount, 2);
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(kNodeCount, 2);

    // Populate positions vector
    positions.col(0) = command_->getPosition();
    positions(Act::kHebiPitch, 1) = pitch;
    positions.col(1) *= M_PI / 180;  // convert to rad

    // TODO
    double max_diff_rad = 0;
    for (int i = 0; i < kNodeCount; i++) {
        if (abs(positions(i, 1) - positions(i, 0)) > max_diff_rad) {
            max_diff_rad = abs(positions(i, 1) - positions(i, 0));
        }
    }

    // TODO
    Eigen::VectorXd time(2);
    time << 0, max_diff_rad * 30 / M_PI;

    // Log start time and send movement command
    start_time_ = GetCurrentTimeInSec();
    trajectory_ =
        hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions,
                                                            &velocities,
                                                            &accelerations);
}

/**
 * @brief Clears the active actuator movement command(s).
 */
void LibraHebi::Stop() {
    trajectory_ = nullptr;
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Returns the commanded position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return The joint's commanded position value, in degrees
 */
double LibraHebi::GetCommandPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kPitch:
            ret = command_->getPosition()[Act::kHebiPitch];
            break;
    }

    ret *= 180 / M_PI;  // convert to deg
    return ret;
}

/**
 * @brief Returns the actual position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return The joint's actual position value, in degrees
 */
double LibraHebi::GetFeedbackPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kPitch:
            ret = feedback_->getPosition()[Act::kHebiPitch];
            break;
    }

    ret *= 180 / M_PI;  // convert to deg
    return ret;
}

/**
 * @brief Returns the actual torque value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return The joint's actual torque value, in Newton-meters
 */
double LibraHebi::GetFeedbackEffort(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kPitch:
            ret = feedback_->getEffort()[Act::kHebiPitch];
            break;
    }

    return ret;
}

/**
 * @brief Controls the output of verbose debug text.
 * @param true to enable, false to disable
 */
void LibraHebi::SetDebugMode(bool enabled) {
    debug_mode_ = enabled;
}
