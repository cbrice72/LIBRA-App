/******************************************************************************
 * @file   hebi_thread.cpp
 * @brief  Control class for LIBRA HEBI actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "hebi_thread.h"

// C++ Standard Library Headers
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

// Other Library Headers
#include "Eigen/Core"    // Eigen
#include "log_file.hpp"  // HEBI
#include "lookup.hpp"    // HEBI
#include <QDebug>        // Qt::Core

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Thread Overrides
 * !Actuator Commands (slots)
 */

/* Constants */

// Improving Readability of Conversions

constexpr double kDegToRad = M_PI / 180;
constexpr double kRadToDeg = 180 / M_PI;

// HEBI Functions

constexpr int32_t kTimeout = 3000;  // ms
constexpr double kMaxVel = 0.1;     // rad/s, arbitrary

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param families Families of actuators to search for names in
 * @param names Names of actuators to connect to
 * @param debug_mode Whether verbose debug text should be output
 */
HebiThread::HebiThread(QObject* parent, std::vector<std::string> families,
                       std::vector<std::string> names, const bool& debug_mode)
    : AbstractActuatorThread(parent, debug_mode, Actuator::Type::kHebi),
      families_(std::move(families)),
      names_(std::move(names)),
      group_(nullptr),
      num_actuators_(names_.size()) {
    // Initialize HEBI objects
    command_ = std::make_shared<hebi::GroupCommand>(num_actuators_);
    feedback_ = std::make_shared<hebi::GroupFeedback>(num_actuators_);

    // Define joint order for organizing feedback
    // NOTE: ideally, this shouldn't be defined here since it defeats the purpose
    //       of generalizing actuator control code. It should be inferred/provided
    //       by the user, somehow (but I don't have time to make it pretty, so...)
    /*
    joint_order_ = {Actuator::Joint::kMA, Actuator::Joint::kMB,
                    Actuator::Joint::kJ1, Actuator::Joint::kJ2,
                    Actuator::Joint::kJ3};     // LIBRA-I
    */
    joint_order_ = {Actuator::Joint::kPitch};  // LIBRA-II
}

/**
 * @brief Standard destructor.
 */
HebiThread::~HebiThread() {
    // This call to `Disconnect()` does three things:
    //   1) Ensures actuators come to a complete stop
    //   2) Finishes logging and saves it to a file
    //   3) Ensures the main HEBI object gets cleaned up
    Disconnect();
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {  // local to this file

}  // namespace

/**
 * @brief Convenience function for matching individual actuator feedback to the
 *        corresponding `Actuator::Joint`. Facilitates reporting to `MainWindow`.
 *
 * @param feedback Actuator values (ideally, already converted to desired units)
 * @return std::unordered_map<Actuator::Joint, double>
 *
 * @note The enum vector `joint_order_`, defined in the constructor, should have
 *       the same order as the strings in `names_`. Otherwise, this function
 *       will almost certainly obfuscate debugging efforts!
 *
 * @see MainWindow::HandleActuatorFeedback
 */
std::unordered_map<Actuator::Joint, double> HebiThread::GetFeedbackMap(
    const std::vector<double>& feedback) {
    std::unordered_map<Actuator::Joint, double> feedback_map;
    for (auto i = 0; i < joint_order_.size(); i++) {
        feedback_map[joint_order_[i]] = feedback[i];
    }
    return feedback_map;
}

/**
 * @brief Returns minor status information for all connected actuators.
 *        Target position, actual position, and actual torque (effort) are
 *        provided separately as signals.
 *
 * @return QString Comma-delimited status messages
 *
 * @note See following HEBI C++ API page for full list of available
 * feedback:
 *       https://files.hebi.us/docs/cpp/cpp-3.11.1/classhebi_1_1GroupFeedback.html
 */
QString HebiThread::GetStatus() {
    if (group_ == nullptr) {
        return {QString("Not Connected")};
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);  // 0.01
    ss << std::right;                          // right-align numbers

    for (auto i = 0; i < num_actuators_; i++) {
        // Get values
        auto a_vel = feedback_->getVelocity()[i];  // rad/s

        auto defl = feedback_->getDeflection()[i];              // rad
        auto defl_vel = feedback_->getDeflectionVelocity()[i];  // rad/s

        auto volt = feedback_->getVoltage()[i];                  // V
        auto curr = feedback_->getMotorCurrent()[i];             // A
        auto temp = feedback_->getMotorWindingTemperature()[i];  // C
        // (alternatively, `getBoardTemperature()` for electronics)

        // Perform conversions
        a_vel *= defl_vel;      // to deg/s
        defl *= kRadToDeg;      // to deg
        defl_vel *= kRadToDeg;  // to deg/s

        // Create stringstream entry
        // - std::setw(7) for values to account for [sign][#,3][.][#,2]
        ss << "[" << i << "]\n"
           << "  Actual Velocity:     " << std::setw(7) << a_vel << " deg/s\n"
           << "  Deflection:          " << std::setw(7) << defl << " deg\n"
           << "  Deflection Velocity: " << std::setw(7) << defl_vel
           << " deg/s\n"
           << "  Voltage:             " << std::setw(7) << volt << " V\n"
           << "  Current:             " << std::setw(7) << curr << " A\n"
           << "  Temperature:         " << std::setw(7) << temp << " C\n";
    }

    return QString::fromStdString(ss.str());
}

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main command loop.
 */
void HebiThread::run() {
    if (debug_mode_) {
        qDebug() << "[DEBUG] Initialized HebiThread";
    }

    // Initialize thread variables for efficiency
    Eigen::VectorXd pos_cmd(num_actuators_);
    Eigen::VectorXd vel_cmd(num_actuators_);

    std::vector<double> t_pos(num_actuators_);
    std::vector<double> a_pos(num_actuators_);
    std::vector<double> a_trq(num_actuators_);

    std::chrono::duration<double> time(std::chrono::system_clock::now()
                                       - trajectory_start_time_);

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // Build next step of trajectory
        if (trajectory_ != nullptr) {
            time = std::chrono::system_clock::now() - trajectory_start_time_;
            if (time.count() < trajectory_->getDuration()) {
                trajectory_->getState(time.count(), &pos_cmd, &vel_cmd, nullptr);
                command_->setPosition(pos_cmd);
                command_->setVelocity(vel_cmd);
            }
        }

        // Send command and update feedback object
        if (group_ != nullptr) {
            group_->sendCommand(*command_);
            group_->getNextFeedback(*feedback_);
        }

        // Report important statuses individually
        // NOTE: we want vectors of doubles for ease of use, but GroupFeedback's
        //       `get` functions return Eigen types. We use Eigen's `Map` to
        //       convert its `VectorXd` to a `std::vector` without copying.
        Eigen::Map<Eigen::VectorXd>(t_pos.data(), t_pos.size()) =
            feedback_->getPositionCommand() *= kRadToDeg;  // also convert to deg
        emit ReportFeedback(GetFeedbackMap(t_pos),
                            Actuator::Feedback::kTargetPos);

        Eigen::Map<Eigen::VectorXd>(a_pos.data(), a_pos.size()) =
            feedback_->getPosition() *= kRadToDeg;  // also convert to deg
        emit ReportFeedback(GetFeedbackMap(a_pos),
                            Actuator::Feedback::kActualPos);

        Eigen::Map<Eigen::VectorXd>(a_trq.data(),
                                    a_trq.size()) = feedback_->getEffort();
        emit ReportFeedback(GetFeedbackMap(a_trq),
                            Actuator::Feedback::kActualTorque);

        // Report minor statuses all together
        emit ReportStatus(GetStatus(), type_);

        // Don't overwhelm network
        QThread::msleep(10);  // update 100 times/second (theoretically)
    }
}

//------------------------------------------------------------------------------
// !Actuator Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish connections to all actuators.
 *
 * @return true if successful, false otherwise
 *
 * @see
 * https://github.com/HebiRobotics/hebi-cpp-examples/blob/master/advanced/lookup/lookup_example.cpp
 */
void HebiThread::Connect() {
    // In case there is already an active connection, gracefully terminate it
    Disconnect();

    // Create lookup object and wait for actuator list to populate
    hebi::Lookup lookup;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Check if any actuators were found
    {
        const auto entry_list = lookup.getEntryList();

        if (entry_list->size() == 0) {
            // Early exit
            emit ErrorThrown("HEBI - No actuators found on network!");
            return;
        }

        if (debug_mode_) {
            qDebug()
                << "[DEBUG] HEBI - Found following actuators (Family|Name):";

            for (auto entry : *entry_list) {
                qDebug() << " " << entry.family_ << "|" << entry.name_;
            }
        }
    }

    // Filter lookup for relevant actuator(s)
    // NOTE: don't save to class member `group_` until checks have passed
    auto group = lookup.getGroupFromNames(families_, names_, kTimeout);
    if (group == nullptr) {
        emit ErrorThrown("HEBI - Requested actuator families/names not found!");
        return;
    }

    // Load safety parameters
    if (!command_->readSafetyParameters("./bin/shared/hebi/safety.xml")) {
        emit ErrorThrown("HEBI - Failed to load safety parameters!");
        return;
    }

    // Load gains
    if (!command_->readGains("./bin/shared/hebi/gains.xml")) {
        emit ErrorThrown("HEBI - Failed to load gain parameters!");
        return;
    }

    // Initialize actuator(s) with above parameters
    if (!group->sendCommandWithAcknowledgement(*command_, kTimeout)) {
        emit ErrorThrown("HEBI - Didn't receive acknowledgement from "
                         "actuator initialization!");
        return;
    }
    command_->clear();

    // Command actuator(s) to hold current position
    group_ = group;
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());

    // Start logging
    const std::string log_path = group_->startLog("./log");
    if (log_path.empty()) {
        emit ErrorThrown("HEBI - Log directory (log/) does not exist in CWD!");
        return;
    }

    if (debug_mode_) {
        qDebug() << "[DEBUG] HEBI - Creating log file at" << log_path;
    }
}

/**
 * @brief Terminates the active connection(s).
 *
 * @return true if successful, false otherwise
 */
void HebiThread::Disconnect() {
    // In case this was called in the middle of a movement, gracefully stop
    Stop();

    if (group_ != nullptr) {
        // Stop logging
        group_->stopLog();

        // Destructing hebi::Group automatically cleans it up
        group_.reset();
    }
}

/**
 * @brief Sets movement targets/trajectories for all connected actuators.
 *
 * @param target Target angles (absolute) for all actuators, in degrees
 */
void HebiThread::SetTarget(const std::vector<double>& target) {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't move actuators; not connected!");
        return;
    }

    // Validate input
    if (target.size() != num_actuators_) {
        emit ErrorThrown("HEBI - Size of command vector != number of "
                         "connected actuators!");
        return;
    }

    // Make position, velocity, and acceleration commands for start & end points
    Eigen::MatrixXd pos(num_actuators_, 2);
    // Eigen::MatrixXd vel = Eigen::MatrixXd::Constant(num_actuators_, 2, kMaxVel);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(num_actuators_, 2);  // default
    Eigen::MatrixXd accel = Eigen::MatrixXd::Zero(num_actuators_, 2);  // default

    // Populate positions
    pos.col(0) = command_->getPosition();  // start (current value)
    for (auto i = 0; i < target.size(); i++) {
        pos(i, 1) = target.at(i) * kDegToRad;  // end (target value)
    }

    // Determine greatest change in position for calculating trajectory times
    double max_difference = 0;
    for (auto i = 0; i < num_actuators_; i++) {
        max_difference = std::max(abs(pos(i, 1) - pos(i, 0)), max_difference);
    }

    // Calculate trajectory start and end times
    Eigen::VectorXd time(2);
    time << 0, max_difference / kMaxVel;

    // Log start time and create trajectory
    trajectory_start_time_ = std::chrono::system_clock::now();
    trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(time, pos,
                                                                      &vel,
                                                                      &accel);
}

/**
 * @brief Halts the trajectories of all actuators.
 */
void HebiThread::Stop() {
    if (group_ == nullptr) {
        return;  // do nothing
    }

    trajectory_ = nullptr;
}
