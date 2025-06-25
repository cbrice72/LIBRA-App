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
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

// Other Library Headers
#include "Eigen/Core"    // Eigen
#include "log_file.hpp"  // HEBI
#include "lookup.hpp"    // HEBI
#include <QDir>          // Qt::Core

// Project Headers
#include "ros2_logger.h"

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Thread Overrides
 * !Actuator Commands (slots)
 */

// Improving Readability of Conversions

constexpr double kDegToRad = M_PI / 180;
constexpr double kRadToDeg = 180 / M_PI;

// HEBI Functions

constexpr int32_t kTimeout = 3000;   // ms
constexpr double kMaxVel = 0.1;      // rad/s
constexpr double kStiffness = 50.0;  // Nm/rad
constexpr double kDamping = 1.0;     // Nm/rad/s

// LIBRA Control

constexpr double kTorqueCompUpperBound = 6.0;  // Nm
constexpr double kTorqueCompLowerBound = 3.0;  // Nm

// Status Message Formatting

constexpr int kLabelWidth = 21;  // longest label = 19 char, +2 for visuals
constexpr int kValueWidth = 8;  // format is [sign][#,3][.][#,2], +1 for visuals

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Formats actuator values and appends them to an existing stream.
 *
 * @param ss Reference to an output stream
 * @param label Row title
 * @param values Values for every actuator
 * @param unit Value units, if applicable
 */
void AppendRow(std::ostringstream& ss, const std::string& label,
               const std::vector<double>& values,
               const std::string& unit = "") {
    ss << std::left << std::setw(kLabelWidth) << label;  // left-aligned

    for (double val : values) {
        ss << std::right << std::setw(kValueWidth) << std::fixed
           << std::setprecision(1) << val;  // 0.1, right-aligned
    }

    if (!unit.empty()) {
        ss << " " << unit;
    }

    ss << "\n";
}

}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

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
      num_actuators_(names_.size()),
      trajectory_start_time_(std::chrono::system_clock::now())
#ifdef BUILD_WITH_ROS2
      ,
      rclcpp::Node("hebi_node")
#endif
{
    // Initialize the logger
#ifdef BUILD_WITH_ROS2
    logger_ = std::make_unique<Ros2Logger>(debug_mode_, this->get_logger());
#else
    logger_ = std::make_unique<QtLogger>(debug_mode_);
#endif

    // Initialize HEBI objects
    command_ = std::make_shared<hebi::GroupCommand>(num_actuators_);
    feedback_ = std::make_shared<hebi::GroupFeedback>(num_actuators_);

    // Define joint order for organizing feedback
    // NOTE: ideally, this shouldn't be defined here since it defeats the purpose
    //       of generalizing actuator control code. It should be inferred/provided
    //       by the user, somehow (but I don't have time to make it pretty, so...)
#if LIBRA_VERSION == 1
    joint_order_ = {Actuator::Joint::kMA, Actuator::Joint::kMB,
                    Actuator::Joint::kJ1, Actuator::Joint::kJ2,
                    Actuator::Joint::kJ3};  // LIBRA-I
#elif LIBRA_VERSION == 2
    joint_order_ = {Actuator::Joint::kPitch};  // LIBRA-II
#endif

    // Resize status vectors to match number of actuators
    status_a_vel_.resize(num_actuators_);
    status_defl_.resize(num_actuators_);
    status_defl_vel_.resize(num_actuators_);
    status_volt_.resize(num_actuators_);
    status_curr_.resize(num_actuators_);
    status_temp_.resize(num_actuators_);

#ifdef BUILD_WITH_ROS2
    // Initialize ROS2 components
    state_pub_ = this->create_publisher<msgHebiState>("hebi/state", 10);

    state_msg_.header.frame_id = "hebi_actuators";
    state_msg_.families = families_;
    state_msg_.names = names_;

    // Resize message vectors to match number of actuators
    state_msg_.target_pos.resize(num_actuators_);
    state_msg_.actual_pos.resize(num_actuators_);
    state_msg_.target_vel.resize(num_actuators_);
    state_msg_.actual_vel.resize(num_actuators_);
    state_msg_.target_trq.resize(num_actuators_);
    state_msg_.actual_trq.resize(num_actuators_);
    state_msg_.deflection.resize(num_actuators_);
    state_msg_.deflection_vel.resize(num_actuators_);
    state_msg_.voltage.resize(num_actuators_);
    state_msg_.current.resize(num_actuators_);
    state_msg_.motor_temp.resize(num_actuators_);
#endif
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

    logger_->Debug("Cleaned up HebiThread");
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Convenience function for matching individual actuator feedback to the
 *        corresponding `Actuator::Joint`. Facilitates reporting to `MainWindow`.
 *
 * @param feedback Actuator values (ideally, already converted to desired units)
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
QString HebiThread::GetStatus() const {
    if (group_ == nullptr) {
        return QString("Not Connected");
    }

    std::ostringstream ss;

    // Get references to minor status info vectors (improves readability)
    const auto& vel = feedback_->getVelocity();
    const auto& defl = feedback_->getDeflection();
    const auto& defl_vel = feedback_->getDeflectionVelocity();
    const auto& volt = feedback_->getVoltage();
    const auto& curr = feedback_->getMotorCurrent();
    const auto& temp = feedback_->getMotorWindingTemperature();
    // (alternatively, `getBoardTemperature()` for electronics)

    // Convert data, if necessary
    for (int i = 0; i < num_actuators_; ++i) {
        status_a_vel_[i] = vel[i] * kRadToDeg;
        status_defl_[i] = defl[i] * kRadToDeg;
        status_defl_vel_[i] = defl_vel[i] * kRadToDeg;
        status_volt_[i] = volt[i];
        status_curr_[i] = curr[i];
        status_temp_[i] = temp[i];
    }

    // Format header row
    ss << std::setw(kLabelWidth) << "Actuator Name";
    for (int i = 0; i < num_actuators_; ++i) {
        ss << std::right << std::setw(kValueWidth) << "[" + names_[i] + "]";
    }
    ss << "\n";

    // Format information rows
    AppendRow(ss, "Actual Velocity", status_a_vel_, "deg/s");
    AppendRow(ss, "Deflection", status_defl_, "deg");
    AppendRow(ss, "Deflection Velocity", status_defl_vel_, "deg/s");
    AppendRow(ss, "Voltage", status_volt_, "V");
    AppendRow(ss, "Current", status_curr_, "A");
    AppendRow(ss, "Temperature", status_temp_, "C");

    return QString::fromStdString(ss.str());
}

#ifdef BUILD_WITH_ROS2
/**
 * @brief Publish HEBI actuator state(s).
 *
 * @note Currently, this function only exists to log actuator information in the
 *       ROS2 ecosystem (i.e., via rosbag).
 */
void HebiThread::PublishState() {
    if (!state_pub_ || group_ == nullptr) {
        return;
    }

    // Update header timestamp
    state_msg_.header.stamp = this->get_clock()->now();

    // Populate the message and publish it
    for (int i = 0; i < num_actuators_; ++i) {
        // Positions (convert from rad to deg)
        state_msg_.target_pos[i] = feedback_->getPositionCommand()[i]
                                   * kRadToDeg;
        state_msg_.actual_pos[i] = feedback_->getPosition()[i] * kRadToDeg;

        // Velocities (convert from rad/s to deg/s)
        state_msg_.target_vel[i] = feedback_->getVelocityCommand()[i]
                                   * kRadToDeg;
        state_msg_.actual_vel[i] = feedback_->getVelocity()[i] * kRadToDeg;

        // Efforts/Torques
        state_msg_.target_trq[i] = feedback_->getEffortCommand()[i];
        state_msg_.actual_trq[i] = feedback_->getEffort()[i];

        // Additional sensor data
        state_msg_.deflection[i] = feedback_->getDeflection()[i] * kRadToDeg;
        state_msg_.deflection_vel[i] = feedback_->getDeflectionVelocity()[i]
                                       * kRadToDeg;

        state_msg_.voltage[i] = feedback_->getVoltage()[i];
        state_msg_.current[i] = feedback_->getMotorCurrent()[i];

        state_msg_.motor_temp[i] = feedback_->getMotorWindingTemperature()[i];
    }

    state_pub_->publish(state_msg_);
}
#endif

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main command loop.
 */
void HebiThread::run() {
    logger_->Debug("Initialized HebiThread");

    // Initialize thread variables for efficiency
    Eigen::VectorXd pos_cmd(num_actuators_);
    Eigen::VectorXd vel_cmd(num_actuators_);
    // Eigen::VectorXd trq_cmd(num_actuators_);  // not currently used

    std::vector<double> t_pos(num_actuators_);
    std::vector<double> a_pos(num_actuators_);
    std::vector<double> a_trq(num_actuators_);

    double arm_torque_r{0};      // magnitude of torque exerted on central joint
    double arm_torque_theta{0};  // angle of torque exerted on central joint

    std::chrono::duration<double> time(std::chrono::system_clock::now()
                                       - trajectory_start_time_);

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        if (group_ == nullptr) {
            emit ReportStatus(GetStatus(), type_);  // "Not Connected"
            QThread::msleep(10);
            continue;
        }

        // Update feedback object
        group_->getNextFeedback(*feedback_);

        // Control the overall torque experienced by the central joint
        // (LIBRA-I: 2-DoF joint, LIBRA-II: pitch joint)
        if (torque_control_en_) {
            // TODO: the current "algorithm" is just simple hysteresis which
            //       stops ALL arm movement until the counterweight is full
            //       enough. This is not ideal, and we should be predicting how
            //       arm movement will affect the central joint torque.

            // Retrieve torque magnitude and direction (i.e., polar coords)
#if LIBRA_VERSION == 1
            arm_torque_r =
                std::max(std::abs(feedback_->getEffort()[Actuator::Joint::kMA]),
                         std::abs(feedback_->getEffort()[Actuator::Joint::kMB]));

            arm_torque_theta = std::atan2(
                -feedback_->getEffort()[Actuator::Joint::kMA]
                    + feedback_->getEffort()[Actuator::Joint::kMB],  // pitch
                -feedback_->getEffort()[Actuator::Joint::kMA]
                    - feedback_->getEffort()[Actuator::Joint::kMB]);  // roll
#elif LIBRA_VERSION == 2
            arm_torque_r = std::abs(
                feedback_->getEffort()[Actuator::Joint::kPitch]);
            arm_torque_theta = (arm_torque_r >= 0) ? 0.0 : M_PI;
#endif

            if (arm_torque_r > kTorqueCompUpperBound) {
                // Disable movement if arm torque is too high
                movement_en_ = false;
            }

            if (arm_torque_r >= kTorqueCompLowerBound) {
                // While arm torque remains above the specified lower bound, report
                // magnitude and direction (if applicable) to arduino_thread
                emit ReportArmTorque(arm_torque_theta);
            } else {
                // Re-enable movement when arm torque reaches an acceptable level
                movement_en_ = true;
            }
        }

        // Determine movement command
        if (movement_en_ && trajectory_ != nullptr) {
            time = std::chrono::system_clock::now() - trajectory_start_time_;
            if (time.count() < trajectory_->getDuration()) {
                // Build next step of trajectory
                trajectory_->getState(time.count(), &pos_cmd, &vel_cmd, nullptr);
                command_->setPosition(pos_cmd);
                command_->setVelocity(vel_cmd);
            } else {
                // Trajectory is complete
                trajectory_.reset();

                logger_->Debug("HEBI - Trajectory complete");
            }
        } else {
            // Add compensating effort/torque to resist external forces
            Eigen::VectorXd effort = Eigen::VectorXd::Zero(num_actuators_);
            for (int i = 0; i < num_actuators_; i++) {
                // Virtual spring-damper
                auto pos_error = command_->getPosition()[i]
                                 - feedback_->getPosition()[i];
                auto vel_damping = -feedback_->getVelocity()[i];
                effort(i) = (kStiffness * pos_error) + (kDamping * vel_damping);
            }
            command_->setEffort(effort);

            // NOTE: this is problematic since it counteracts the actuator's
            //       internal PID controller (see gains.xml)
            /*
            // Counter measured angular velocity
            vel_cmd = -feedback_->getGyro().col(2);  // z-axis (same as output)
            command_->setVelocity(vel_cmd);
            */
        }

        // Send movement command
        group_->sendCommand(*command_);

#ifdef BUILD_WITH_ROS2
        // Make actuator state info available to other ROS2 nodes
        PublishState();
#endif

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

        // Report miscellaneous statuses in one batch
        emit ReportStatus(GetStatus(), type_);

        // Don't overwhelm network
        QThread::msleep(10);  // 100 Hz
    }
}

//------------------------------------------------------------------------------
// !Actuator Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish connections to all actuators.
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

        logger_->Debug("HEBI - Found following actuators (Family|Name):");
        for (auto entry : *entry_list) {
            logger_->Debug("  " + entry.family_ + " | " + entry.name_);
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

    logger_->Debug("HEBI - Connection successful");
    emit Connected(true);

    // Command actuator(s) to hold current position
    group_ = group;
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());

    // Ensure a log directory exists (or else group_->startLog()  will fail!)
    QDir log_dir("./log");
    if (!log_dir.exists()) {
        if (!log_dir.mkpath(".")) {
            emit ErrorThrown("HEBI - Failed to create log directory");
            return;
        }
    }

    // Start logging
    const std::string log_path = group_->startLog("./log");
    if (log_path.empty()) {
        emit ErrorThrown("HEBI - Log directory (log/) does not exist in CWD!");
        return;
    }

    logger_->Debug("HEBI - Creating log file at" + log_path);
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

        logger_->Debug("HEBI - Gracefully disconnected from actuator(s)");
        emit Connected(false);
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

    std::stringstream trajectory_ss;  // for debug only

    // Populate positions
    group_->getNextFeedback(*feedback_);
    pos.col(0) = feedback_->getPosition();  // start (current value)
    for (auto i = 0; i < target.size(); i++) {
        pos(i, 1) = target.at(i) * kDegToRad;  // end (target value)
        if (debug_mode_) {
            trajectory_ss << std::to_string(pos(i, 1)) << "";
        }
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

    logger_->Debug("HEBI - Set trajectory target(s) to " + trajectory_ss.str()
                   + " rad");
}

/**
 * @brief Halts the trajectories of all actuators.
 */
void HebiThread::Stop() {
    if (group_ == nullptr) {
        return;  // do nothing
    }

    trajectory_.reset();

    logger_->Debug("HEBI - Trajectory reset");
}

/**
 * @brief TODO: documentation.
 */
void HebiThread::SetAutoTorqueComp(const bool& enabled) {
    torque_control_en_ = enabled;

    if (!torque_control_en_) {
        // "Manual" mode: only force allow movement if torque compensation is disabled
        movement_en_ = true;
    }
}
