/******************************************************************************
 * @file   arm_thread.cpp
 * @brief  Control code for LIBRA-II arm actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "arm_thread.h"

// C++ Standard Library Headers
#include <chrono>
#include <iostream>

// Other Library Headers
#include <QWidget>  // Qt::Widgets

// Project Headers
//   (none)

constexpr uint8_t kHEBINodeCount = 1;  // total number of HEBI actuators
constexpr uint8_t kEPOSNodeCount = 1;  // total number of EPOS (Maxon) actuators

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Actuator Commands
 * !Getters & Setters
 */

/**
 * @brief Standard constructor.
 */
ArmThread::ArmThread() : start_time_(GetCurrentTimeInSec()) {
    command_ = std::make_unique<hebi::GroupCommand>(kHEBINodeCount);
    feedback_ = std::make_unique<hebi::GroupFeedback>(kHEBINodeCount);

    if (!Connect()) {
        qWarning() << "[WARN] Initializing without HEBI actuator(s).";
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Retrieves the current system time (i.e., time since epoch) in seconds.
 *
 * @return Current system time in seconds
 */
std::chrono::system_clock::rep ArmThread::GetCurrentTimeInSec() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(now).count();
}

/**
 * @brief TODO: documentation.
 */
void ArmThread::run() {
    // Initialize HEBI feedback containers prior to thread start for efficiency
    std::array<double, kHEBINodeCount> target;
    std::array<double, kHEBINodeCount> actual;
    std::array<double, kHEBINodeCount> torque;

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // --- EPOS (Maxon) Actuators ---

        // TODO: implementation

        // --- HEBI Actuators ---

        command_->setVelocity(Eigen::VectorXd::Zero(kHEBINodeCount));

        // Calculate trajectories
        if (trajectory_ != nullptr) {
            const double time = (GetCurrentTimeInSec() - start_time_) / 1000.0;
            if (trajectory_->getDuration() > time) {
                Eigen::VectorXd pos_cmd(kHEBINodeCount);
                Eigen::VectorXd vel_cmd(kHEBINodeCount);
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
            for (auto i = 0; i < kNodeCount; i++) {
                auto joint = static_cast<ArmThread::Joint>(i);
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
 * @return true if connection and initialization succeeded; false Otherwise
 */
bool ArmThread::Connect() {
    // --- EPOS (Maxon) Actuators ---

    // TODO: implementation

    // --- HEBI Actuators ---

    hebi::Lookup lookup;
    group_ = lookup.getGroupFromNames({"X8-16"}, {"Pitch"});
    if (group_ == nullptr) {
        std::cerr << "[ERROR] HEBI - Failed to connect to actuators!\n";
        command_->setPosition(Eigen::VectorXd::Zero(kHEBINodeCount));
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
 * @brief TODO: documentation.
 *
 * @param yaw Desired yaw angle (deg)
 * @param pitch Desired pitch angle (deg)
 */
void ArmThread::Move(double yaw, double pitch) {
    // --- EPOS (Maxon) Actuators ---

    // TODO: implementation

    // --- HEBI Actuators ---

    // TODO: if HEBI actuator exists, do the following
    {
        Eigen::MatrixXd positions(kHEBINodeCount, 2);
        Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(kHEBINodeCount, 2);
        Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(kHEBINodeCount, 2);

        // Populate positions vector
        positions.col(0) = command_->getPosition();
        positions(kPitchID, 1) = pitch;
        positions.col(1) *= M_PI / 180;  // convert to rad

        // TODO: explanation
        double max_diff_rad = 0;
        for (int i = 0; i < kHEBINodeCount; i++) {
            if (abs(positions(i, 1) - positions(i, 0)) > max_diff_rad) {
                max_diff_rad = abs(positions(i, 1) - positions(i, 0));
            }
        }

        // TODO: explanation
        Eigen::VectorXd time(2);
        time << 0, max_diff_rad * 30 / M_PI;

        // Log start time and send movement command
        start_time_ = GetCurrentTimeInSec();
        trajectory_ =
            hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions,
                                                                &velocities,
                                                                &accelerations);
    }
}

/**
 * @brief Clears the active actuator movement command(s).
 */
void ArmThread::Stop() {
    // --- EPOS (Maxon) Actuators ---

    // TODO: implementation

    // --- HEBI Actuators ---

    // TODO: if HEBI actuator exists, do the following
    { trajectory_ = nullptr; }
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Returns the commanded position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return The joint's commanded position value (deg)
 */
double ArmThread::GetCommandPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kYaw:
            // TODO: implementation
            break;
        case kPitch:
            ret = command_->getPosition()[kPitchID];
            ret *= 180 / M_PI;  // convert to deg
            break;
    }

    return ret;
}

/**
 * @brief Returns the actual position value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return The joint's actual position value (deg)
 */
double ArmThread::GetFeedbackPosition(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kYaw:
            // TODO: implementation
            break;
        case kPitch:
            ret = feedback_->getPosition()[kPitchID];
            ret *= 180 / M_PI;  // convert to deg
            break;
    }

    return ret;
}

/**
 * @brief Returns the actual torque value for the specified joint.
 *
 * @param joint The LIBRA joint to query
 * @return The joint's actual torque value (N-m)
 */
double ArmThread::GetFeedbackEffort(Joint joint) {
    double ret = 0;

    switch (joint) {
        case kYaw:
            // TODO: implementation
            break;
        case kPitch:
            ret = feedback_->getEffort()[kPitchID];
            break;
    }

    return ret;
}

/**
 * @brief Sets whether or not verbose debug text is displayed
 *
 * @param true to enable, false to disable
 */
void ArmThread::SetDebugMode(bool enabled) {
    debug_mode_ = enabled;
}
