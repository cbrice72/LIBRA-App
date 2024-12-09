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

constexpr double kSecondsPerMin = 60;
constexpr double kRadPerRevolution = 2 * M_PI;

constexpr double kDegToRad = M_PI / 180;
constexpr double kRadToDeg = 180 / M_PI;

// HEBI Functions

constexpr int32_t kTimeout = 3000;  // ms

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
    : AbstractActuatorThread(parent, debug_mode),
      families_(std::move(families)),
      names_(std::move(names)),
      group_(nullptr),
      num_actuators_(names_.size()) {
    // Initialize HEBI objects
    command_ = std::make_shared<hebi::GroupCommand>(num_actuators_);
    feedback_ = std::make_shared<hebi::GroupFeedback>(num_actuators_);
}

/**
 * @brief Standard destructor.
 */
HebiThread::~HebiThread() {
    if (group_ != nullptr) {
        // Stop logging
        group_->stopLog();
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {  // local to this file

}  // namespace

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
std::vector<QString> HebiThread::GetStatus() {
    if (group_ == nullptr) {
        return {QString("Not Connected")};
    }

    std::vector<QString> statuses;

    for (auto i = 0; i < num_actuators_; i++) {
        // Get values
        auto actual_vel = feedback_->getVelocity()[i];  // rad/s

        auto defl = feedback_->getDeflection()[i];              // mm?
        auto defl_vel = feedback_->getDeflectionVelocity()[i];  // mms?

        auto volt = feedback_->getVoltage()[i];                  // V
        auto curr = feedback_->getMotorCurrent()[i];             // A
        auto temp = feedback_->getMotorWindingTemperature()[i];  // C
        // alternatively, getBoardTemperature() for electronics

        // Perform conversions
        actual_vel *= kSecondsPerMin / kRadPerRevolution;  // to rpm

        // Create "entry" in stringstream
        std::stringstream status_ss;
        status_ss << "[" << i << "]"
                  << "\n  Actual Velocity (rpm): " << actual_vel
                  << "\n  Deflection (mm): " << defl
                  << "\n  Deflection Velocity (mm/s): " << defl_vel
                  << "\n  Voltage (V): " << volt << "\nCurrent (A): " << curr
                  << "\n  Temperature (C): " << temp;

        statuses.push_back(QString::fromStdString(status_ss.str()));
    }

    return statuses;
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
                Eigen::VectorXd pos_cmd(num_actuators_);
                Eigen::VectorXd vel_cmd(num_actuators_);
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
        // NOTE: we want vectors of doubles, but GroupFeedback's `get` functions
        //       return Eigen types. We use Eigen's `Map` to convert its
        //       `VectorXd` to a `std::vector` without copying.
        Eigen::Map<Eigen::VectorXd>(t_pos.data(), t_pos.size()) =
            feedback_->getPositionCommand() *= kRadToDeg;
        emit ReportTargetPos(t_pos);

        Eigen::Map<Eigen::VectorXd>(a_pos.data(), a_pos.size()) =
            feedback_->getPosition() *= kRadToDeg;
        emit ReportActualPos(a_pos);

        Eigen::Map<Eigen::VectorXd>(a_trq.data(),
                                    a_trq.size()) = feedback_->getEffort();
        emit ReportActualTorque(a_trq);

        // Report minor statuses all together
        emit ReportStatus(GetStatus());

        // Don't overwhelm network
        QThread::msleep(100);  // update 10 times/second
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
 * @note For a complete example, see HEBI `hebi-cpp-examples` GitHub:
 *       https://github.com/HebiRobotics/hebi-cpp-examples/blob/master/advanced/lookup/lookup_example.cpp
 */
void HebiThread::Connect() {
    // Create lookup object and wait for actuator list to populate
    hebi::Lookup lookup;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Check if any actuators were found
    {
        const auto entry_list = lookup.getEntryList();

        if (entry_list->size() == 0) {
            // Early exit
            emit ErrorThrown("[ERROR] HEBI - No actuators found on network!");
            return;
        }

        if (debug_mode_) {
            qDebug()
                << "[DEBUG] HEBI - Found following actuators (Family|Name):\n";

            for (auto entry : *entry_list) {
                qDebug() << " " << entry.family_ << "|" << entry.name_;
            }
        }
    }

    // Filter lookup for relevant actuator(s)
    group_ = lookup.getGroupFromNames(families_, names_, kTimeout);
    if (group_ == nullptr) {
        emit ErrorThrown(
            "[ERROR] HEBI - Requested actuator families/names not found!");
        return;
    }

    // Load safety parameters
    if (!command_->readSafetyParameters("bin/shared/hebi/safety.xml")) {
        emit ErrorThrown("[ERROR] HEBI - Failed to load safety parameters!");
        return;
    }

    // Load gains
    if (!command_->readGains("bin/shared/hebi/gains.xml")) {
        emit ErrorThrown("[ERROR] HEBI - Failed to load gain parameters!");
        return;
    }

    // Start logging
    const std::string log_path = group_->startLog("./log");
    if (log_path.empty()) {
        emit ErrorThrown(
            "[ERROR] HEBI - Log directory (log/) does not exist in CWD!");
        return;
    }

    if (debug_mode_) {
        qDebug() << "[DEBUG] HEBI - Creating log file at" << log_path;
    }

    // Initialize actuator(s) with above parameters
    if (!group_->sendCommandWithAcknowledgement(*command_, kTimeout)) {
        emit ErrorThrown("[ERROR] HEBI - Didn't receive acknowledgement from "
                         "actuator initialization!");
        return;
    }
    command_->clear();

    // Command actuator(s) to hold current position
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());
}

/**
 * @brief Terminates the active connection(s).
 *
 * @return true if successful, false otherwise
 */
void HebiThread::Disconnect() {
    if (group_) {
        // Stop logging
        group_->stopLog();

        // Destructing hebi::Group automatically cleans it up
        group_.reset();
    }
}

/**
 * @brief Sends movement commands to all connected actuators.
 *
 * @param deg Target angles (absolute) for all actuators
 *
 * @note If an actuator doesn't directly accept degrees as part of its
 *       movement command (i.e., if two actuators work together to effect two
 *       axes), the translation should be done before calling this function.
 */
void HebiThread::SetTarget(const std::vector<double>& deg) {
    if (group_ == nullptr) {
        emit ErrorThrown("[ERROR] HEBI - Can't move actuators; not connected!");
        return;
    }

    // Make command vectors of two points: current -> target
    Eigen::MatrixXd positions(num_actuators_, 2);
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_actuators_, 2);
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_actuators_, 2);

    // Populate positions vector
    positions.col(0) = command_->getPosition();  // current (start)
    for (auto i = 0; i < deg.size(); i++) {
        positions(i, 1) = deg.at(i);  // target (finish)
    }
    positions.col(1) *= kDegToRad;

    // Determine greatest change in position for calculating trajectory time
    // NOTE: this is just future-proofing; LIBRA-II only has one HEBI actuator
    double max_difference = 0;
    for (auto i = 0; i < num_actuators_; i++) {
        max_difference = std::max(abs(positions(i, 1) - positions(i, 0)),
                                  max_difference);
    }

    // Calculate trajectory start and end times
    Eigen::VectorXd time(2);
    time << 0, max_difference * 12 / M_PI;  // 12 is arbitrary - change at will

    // Log start time and create trajectory
    trajectory_start_time_ = std::chrono::system_clock::now();
    trajectory_ =
        hebi::trajectory::Trajectory::createUnconstrainedQp(time, positions,
                                                            &velocities,
                                                            &accelerations);
}

/**
 * @brief Halts the trajectories of all actuators.
 */
void HebiThread::Stop() {
    if (group_ == nullptr) {
        emit ErrorThrown("[ERROR] HEBI - Can't stop actuators; not connected!");
        return;
    }

    trajectory_ = nullptr;
}
