/******************************************************************************
 * @file   hebi_thread.cpp
 * @brief  Control code for LIBRA arm HEBI actuators; implementation file.
 *         (adapted from Yuto Goto's work)
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "hebi_thread.h"

// C++ Standard Library Headers
#include <chrono>
#include <iostream>

// Other Library Headers
#include <QWidget>  // Qt

// Project Headers
//   (none)

constexpr uint8_t kHebiNodeCount = 5;  // total number of HEBI actuators

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Actuator Commands
 * !Getters & Setters
 */

/**
 * @brief Standard constructor.
 */
HebiThread::HebiThread() : start_time_(GetCurrentTimeInSec()) {
    command_ = std::make_unique<hebi::GroupCommand>(5);
    feedback_ = std::make_unique<hebi::GroupFeedback>(5);

    if (!Connect()) {
        qWarning() << "[WARN] Initializing without HEBI actuators.";
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Retrieves the current system time (i.e., time since epoch) in seconds.
 *
 * @return std::chrono::system_clock::rep Current system time in seconds
 */
std::chrono::system_clock::rep HebiThread::GetCurrentTimeInSec() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(now).count();
}

/**
 * @brief TODO.
 */
void HebiThread::run() {
    // Initialize feedback containers prior to thread start for efficiency
    std::array<double, 5> target;
    std::array<double, 5> actual;
    std::array<double, 5> torque;

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        command_->setVelocity(Eigen::VectorXd::Zero(5));

        // Calculate trajectories
        if (trajectory_ != nullptr) {
            const double time = (GetCurrentTimeInSec() - start_time_) / 1000.0;
            if (trajectory_->getDuration() > time) {
                Eigen::VectorXd pos_cmd(5);
                Eigen::VectorXd vel_cmd(5);
                trajectory_->getState(time, &pos_cmd, &vel_cmd, nullptr);
                command_->setPosition(pos_cmd);
                command_->setVelocity(vel_cmd);
            }
        }

        if (group_ != nullptr) {
            // Send movement commands and update feedback
            group_->sendCommand(*command_);
            group_->getNextFeedback(*feedback_);

            // Retrieve HEBI actuator data
            for (auto i = 0; i < kHebiNodeCount; i++) {
                auto joint = static_cast<HebiThread::Joint>(i);
                target.at(i) = GetCommandPosition(joint);
                actual.at(i) = GetFeedbackPosition(joint);
                torque.at(i) = GetFeedbackEffort(joint);

                // Emit signal
                emit InformState(target, actual, torque);
            }
        }

        QThread::sleep(1);  // TOOD: for testing purposes only
        // QThread::msleep(20);  // update 50 times/second
    }
}

//------------------------------------------------------------------------------
// !Actuator Commands
//------------------------------------------------------------------------------

/**
 * @brief Connects to the HEBI actuator group and sets their parameters.
 *
 * @return true If connection and initialization succeeded
 * @return false Otherwise
 */
bool HebiThread::Connect() {
    hebi::Lookup lookup;
    group_ = lookup.getGroupFromNames({"X8-16"}, {"MA", "MB", "J1", "J2", "J3"});
    if (group_ == nullptr) {
        std::cerr << "[ERROR] HEBI - Failed to connect to actuators!\n";
        command_->setPosition(Eigen::VectorXd::Zero(5));
        return false;
    }

    if (!command_->readSafetyParameters("params/safety.xml")) {
        std::cerr << "[ERROR] HEBI - Failed to load safety parameters!\n";
        return false;
    }

    if (!command_->readGains("params/gain.xml")) {
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
 * @param roll Desired roll angle, in degrees
 * @param pitch Desired pitch angle, in degrees
 * @param j1 Desired J1 (arm yaw) angle, in degrees
 * @param j2 Desired J2 (arm yaw) angle, in degrees
 * @param j3 Desired J3 (arm pitch) angle, in degrees
 */
void HebiThread::Move(double roll, double pitch, double j1, double j2,
                      double j3) {
    Eigen::MatrixXd positions(5, 2);
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(5, 2);
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(5, 2);

    // Populate positions vector
    positions.col(0) = command_->getPosition();
    positions(Act::kHebiMA, 1) = -roll - pitch;  // TODO: improve
    positions(Act::kHebiMB, 1) = -roll + pitch;  // TODO: improve
    positions(Act::kHebiJ1, 1) = j1;
    positions(Act::kHebiJ2, 1) = -j2;
    positions(Act::kHebiJ3, 1) = j3;
    positions.col(1) *= M_PI / 180;  // convert to rad

    // TODO
    double max_diff_rad = 0;
    for (int i = 0; i < 5; i++) {
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
void HebiThread::Stop() {
    trajectory_ = nullptr;
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Returns the commanded position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return double The joint's commanded position value, in degrees
 */
double HebiThread::GetCommandPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kRoll:
            ret = (-command_->getPosition()[Act::kHebiMA]
                   - command_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kPitch:
            ret = (-command_->getPosition()[Act::kHebiMA]
                   + command_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kJ1:
            ret = command_->getPosition()[Act::kHebiJ1];
            break;
        case kJ2:
            ret = -command_->getPosition()[Act::kHebiJ2];
            break;
        case kJ3:
            ret = command_->getPosition()[Act::kHebiJ3];
            break;
    }

    ret *= 180 / M_PI;  // convert to deg
    return ret;
}

/**
 * @brief Returns the actual position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return double The joint's actual position value, in degrees
 */
double HebiThread::GetFeedbackPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kRoll:
            ret = (-feedback_->getPosition()[Act::kHebiMA]
                   - feedback_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kPitch:
            ret = (-feedback_->getPosition()[Act::kHebiMA]
                   + feedback_->getPosition()[Act::kHebiMB])
                  / 2;
            break;
        case kJ1:
            ret = feedback_->getPosition()[Act::kHebiJ1];
            break;
        case kJ2:
            ret = -feedback_->getPosition()[Act::kHebiJ2];
            break;
        case kJ3:
            ret = feedback_->getPosition()[Act::kHebiJ3];
            break;
    }

    ret *= 180 / M_PI;  // convert to deg
    return ret;
}

/**
 * @brief Returns the actual torque value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return double The joint's actual torque value, in Newton-meters
 */
double HebiThread::GetFeedbackEffort(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kRoll:
            ret = -feedback_->getEffort()[Act::kHebiMA]
                  - feedback_->getEffort()[Act::kHebiMB];
            break;
        case kPitch:
            ret = -feedback_->getEffort()[Act::kHebiMA]
                  + feedback_->getEffort()[Act::kHebiMB];
            break;
        case kJ1:
            ret = feedback_->getEffort()[Act::kHebiJ1];
            break;
        case kJ2:
            ret = -feedback_->getEffort()[Act::kHebiJ2];
            break;
        case kJ3:
            ret = feedback_->getEffort()[Act::kHebiJ3];
            break;
    }

    return ret;
}

/**
 * @brief Returns the actual torque value for the 2-DoF Joint's "A" actuator.
 *
 * @return double The actuator's actual torque value, in Newton-meters
 */
double HebiThread::GetFeedbackEffortMA() {
    return feedback_->getEffort()[Act::kHebiMA];
}

/**
 * @brief Returns the actual torque value for the 2-DoF Joint's "B" actuator.
 *
 * @return double The actuator's actual torque value, in Newton-meters
 */
double HebiThread::GetFeedbackEffortMB() {
    return feedback_->getEffort()[Act::kHebiMB];
}

/**
 * @brief Sets whether or not verbose debug text is displayed
 * @param true to enable, false to disable
 */
void HebiThread::SetDebugMode(bool enabled) {
    debug_mode_ = enabled;
}
