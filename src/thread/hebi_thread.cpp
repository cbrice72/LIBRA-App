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
#include "log_file.hpp"  // HEBI
#include "lookup.hpp"    // HEBI
#include <QDir>          // Qt::Core

// Project Headers
#ifdef BUILD_WITH_ROS2
# include "ros2_logger.h"
#else
# include "qt_logger.h"
#endif

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
 * @param debug_mode Whether to output verbose debug text
 */
HebiThread::HebiThread(QObject* parent, std::vector<std::string> families,
                       std::vector<std::string> names, const bool& debug_mode)
    : AbstractActuatorThread(parent, debug_mode, Actuator::Type::kHebi),
      families_(std::move(families)),
      names_(std::move(names)),
      group_(nullptr),
      n_actuators_(names_.size()),
      trajectory_start_time_(std::chrono::steady_clock::now())
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
    command_ = std::make_shared<hebi::GroupCommand>(n_actuators_);
    feedback_ = std::make_shared<hebi::GroupFeedback>(n_actuators_);

    // Define joint order for organizing feedback
    assert(names_.size() == joint_order_.size());
    for (std::size_t i = 0; i < names_.size(); ++i) {
        joint_order_[i] = Actuator::StringToNameEnum(names_[i]);
    }

    // Resize vectors to match number of actuators
    model_masses_.resize(n_actuators_);
    status_a_vel_.resize(n_actuators_);
    status_defl_.resize(n_actuators_);
    status_defl_vel_.resize(n_actuators_);
    status_volt_.resize(n_actuators_);
    status_curr_.resize(n_actuators_);
    status_temp_.resize(n_actuators_);

#ifdef BUILD_WITH_ROS2
    // Initialize ROS2 components
    state_pub_ = this->create_publisher<msgJointState>("/joint_states", 10);

    state_msg_.name = names_;

    // Resize additional vectors to match number of actuators
    state_msg_.position.resize(n_actuators_);
    state_msg_.velocity.resize(n_actuators_);
    state_msg_.effort.resize(n_actuators_);
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
 * @brief Check torque on the central joint and enforce movement limits.
 *        (central joint = 2-DoF joint (LIBRA-I) -or- pitch joint (LIBRA-II).)
 */
void HebiThread::CheckTorqueControl() {
    if (!torque_control_en_) {
        return;
    }

    // Retrieve torque magnitude and direction (i.e., polar coords)
    const auto& eff = feedback_->getEffort();

#if LIBRA_VERSION == 1
    const double arm_torque_r = std::max(std::abs(eff[Actuator::Name::kMA]),
                                         std::abs(eff[Actuator::Name::kMB]));

    const double arm_torque_theta =
        std::atan2(-eff[Actuator::Name::kMA]
                       + eff[Actuator::Name::kMB],  // pitch
                   -eff[Actuator::Name::kMA]
                       - eff[Actuator::Name::kMB]);  // roll
#elif LIBRA_VERSION == 2
    const double arm_torque_r = std::abs(eff[Actuator::Name::kPitch]);
    const double arm_torque_theta = (arm_torque_r >= 0) ? 0.0 : M_PI;
#endif

    // Apply hysteresis logic
    // TODO: the current "algorithm" is just simple hysteresis which
    //       stops ALL arm movement until the counterweight is full
    //       enough. This is not ideal, and we should be predicting how
    //       arm movement will affect the central joint torque.
    if (arm_torque_r > torque_comp_upper_bound_) {
        // Disable movement if arm torque is too high
        movement_en_ = false;
    }

    if (arm_torque_r >= torque_comp_lower_bound_) {
        // While arm torque remains above the specified lower bound,
        // report direction of torque to arduino_thread
        emit ReportArmTorque(arm_torque_theta);
    } else {
        // Re-enable movement when arm torque reaches an acceptable level
        movement_en_ = true;
    }
}

/**
 * @brief Execute trajectory following or position hold with dynamic/gravity
 *        compensation.
 *
 * @param dt Time since last command loop
 * @param cmd_pos Position command buffer
 * @param cmd_vel Velocity command buffer
 * @param cmd_acc Acceleration command buffer
 * @param cmd_eff Effort command buffer
 *
 * @note Command buffers are reused to avoid repeated allocations at 100 Hz.
 */
void HebiThread::ExecuteMovement(std::chrono::duration<double> dt,
                                 Eigen::VectorXd& cmd_pos,
                                 Eigen::VectorXd& cmd_vel,
                                 Eigen::VectorXd& cmd_acc,
                                 Eigen::VectorXd& cmd_eff) {
    // Initialize persistent variables
    static std::chrono::duration<double> t_trajectory(
        std::chrono::steady_clock::now() - trajectory_start_time_);
    static const Eigen::Vector3d gravity_vec(0, 0, -9.81);

    // Determine movement command
    if (movement_en_ && trajectory_ != nullptr) {
        t_trajectory = std::chrono::steady_clock::now()
                       - trajectory_start_time_;

        if (t_trajectory.count() < trajectory_->getDuration()) {
            // --- MODE 1: TRAJECTORY FOLLOWING ---

            // Build next step of trajectory
            trajectory_->getState(t_trajectory.count(), &cmd_pos, &cmd_vel,
                                  &cmd_acc);
            command_->setPosition(cmd_pos);
            command_->setVelocity(cmd_vel);

            // Calculate effort commands to assist with trajectory tracking
            if (model_based_comp_en_ && model_ != nullptr) {
                // TODO: test this!!
                model_->getDynamicCompEfforts(feedback_->getPosition(), cmd_pos,
                                              cmd_vel, cmd_acc, cmd_eff,
                                              dt.count());
                command_->setEffort(cmd_eff);
            }
        } else {
            // --- MODE 2: POSITION HOLDING ---

            // Trajectory is complete
            trajectory_.reset();

            logger_->Debug("HEBI - Trajectory complete");

            // Calculate effort commands to counteract gravity
            if (model_based_comp_en_ && model_ != nullptr) {
                // TODO: test this!!
                // NOTE: this may be problematic... I deleted it for a
                //       reason (although previously there was no model)
                model_->getGravCompEfforts(cmd_pos, gravity_vec, cmd_eff);
                command_->setVelocity(Eigen::VectorXd());  // clear
                command_->setEffort(cmd_eff);
            }
        }
    }

    // Send movement command
    group_->sendCommand(*command_);
}

/**
 * @brief Publishes actuator feedback (Qt/ROS2).
 *
 * @note Command buffers are reused to avoid repeated allocations at 100 Hz.
 */
void HebiThread::PublishFeedback() {
    // Initialize persistent containers
    static std::vector<double> t_pos(n_actuators_);
    static std::vector<double> a_pos(n_actuators_);
    static std::vector<double> a_eff(n_actuators_);

    // Report important statuses individually
    // NOTE: we want vectors of doubles for ease of use, but GroupFeedback's
    //       `get` functions return Eigen types. We use Eigen's `Map` to
    //       convert its `VectorXd` to a `std::vector` without copying.
    Eigen::Map<Eigen::VectorXd>(t_pos.data(), n_actuators_) =
        feedback_->getPositionCommand() *= kRadToDeg;  // also convert to deg
    emit ReportFeedback(GetFeedbackMap(t_pos), Actuator::Feedback::kTargetPos);

    Eigen::Map<Eigen::VectorXd>(a_pos.data(), n_actuators_) =
        feedback_->getPosition() *= kRadToDeg;  // also convert to deg
    emit ReportFeedback(GetFeedbackMap(a_pos), Actuator::Feedback::kActualPos);

    Eigen::Map<Eigen::VectorXd>(a_eff.data(),
                                n_actuators_) = feedback_->getEffort();
    emit ReportFeedback(GetFeedbackMap(a_eff),
                        Actuator::Feedback::kActualTorque);

    // Report miscellaneous statuses in one batch
    emit ReportStatus(GetStatus(), type_);

#ifdef BUILD_WITH_ROS2
    // Make actuator state info available to other ROS2 nodes
    PublishState();
#endif
}

/**
 * @brief Convenience function for matching individual actuator feedback to the
 *        corresponding `Actuator::Name`. Facilitates reporting to `MainWindow`.
 *
 * @param feedback Actuator values (ideally, already converted to desired units)
 *
 * @note The enum vector `joint_order_`, defined in the constructor, should have
 *       the same order as the strings in `names_`. Otherwise, this function
 *       will almost certainly obfuscate debugging efforts!
 *
 * @see MainWindow::HandleActuatorFeedback
 */
std::unordered_map<Actuator::Name, double> HebiThread::GetFeedbackMap(
    const std::vector<double>& feedback) {
    std::unordered_map<Actuator::Name, double> feedback_map;
    for (auto i = 0; i < joint_order_.size(); ++i) {
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
        return {"Not Connected"};
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
    for (int i = 0; i < n_actuators_; ++i) {
        status_a_vel_[i] = vel[i] * kRadToDeg;
        status_defl_[i] = defl[i] * kRadToDeg;
        status_defl_vel_[i] = defl_vel[i] * kRadToDeg;
        status_volt_[i] = volt[i];
        status_curr_[i] = curr[i];
        status_temp_[i] = temp[i];
    }

    // Format header row
    ss << std::setw(kLabelWidth) << "Actuator Name";
    for (int i = 0; i < n_actuators_; ++i) {
        ss << std::left << std::setw(kValueWidth) << "[" + names_[i] + "]";
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
 * @brief Publishes HEBI actuator state(s).
 */
void HebiThread::PublishState() {
    if (!state_pub_ || group_ == nullptr) {
        return;
    }

    // Update header timestamp
    state_msg_.header.stamp = this->now();

    // Populate the message and publish it
    for (int i = 0; i < n_actuators_; ++i) {
        state_msg_.position[i] = feedback_->getPosition()[i];
        state_msg_.velocity[i] = feedback_->getVelocity()[i];
        state_msg_.effort[i] = feedback_->getEffort()[i];
    }

    state_pub_->publish(state_msg_);
}
#endif

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main command loop.
 *
 *        Delegates torque checks, movement (trajectory/dynamics), and feedback
 *        publishing to helper functions for readability.
 *
 * @see CheckTorqueControl(), ExecuteMovement(), PublishFeedback()
 */
void HebiThread::run() {
    logger_->Debug("Initialized HebiThread");

    // Initialize buffers for loop efficiency
    Eigen::VectorXd cmd_pos(n_actuators_);
    Eigen::VectorXd cmd_vel(n_actuators_);
    Eigen::VectorXd cmd_acc(n_actuators_);  // <- trajectory (DynamicComp only)
    Eigen::VectorXd cmd_eff(n_actuators_);  // -> command (DynamicComp only)

    // For dynamic compensation (DynamicComp)
    auto last_loop_time = std::chrono::steady_clock::now();

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        if (group_ == nullptr) {
            emit ReportStatus(GetStatus(), type_);  // "Not Connected"
            QThread::msleep(10);
            continue;
        }

        // Calculate time since last loop (only used in DynamicComp)
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - last_loop_time;
        last_loop_time = now;

        // Update feedback object
        group_->getNextFeedback(*feedback_);

        // Execute control logic
        CheckTorqueControl();
        ExecuteMovement(dt, cmd_pos, cmd_vel, cmd_acc, cmd_eff);
        PublishFeedback();

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
    if (!command_->readSafetyParameters(
            "./bin/shared/hebi/safety_conservative.xml")) {
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

    // Load robot kinematics
    model_ = hebi::robot_model::RobotModel::loadHRDF(
        "./bin/shared/hebi/libra.hrdf");
    if (model_ == nullptr) {
        emit ErrorThrown("HEBI - Failed to load HRDF!");
        return;
    }
    model_->getMasses(model_masses_);

    // Command actuator(s) to hold current position
    group_ = group;
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());

    // Ensure a log directory exists (or else group_->startLog()  will fail!)
    const QDir log_dir("./log");
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
 * @return true Disconnection successful
 * @return false Disconnection failed
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
    if (target.size() != n_actuators_) {
        emit ErrorThrown("HEBI - Size of command vector != number of "
                         "connected actuators!");
        return;
    }

    // Populate positions
    Eigen::MatrixXd pos(n_actuators_, 2);

    group_->getNextFeedback(*feedback_);
    pos.col(0) = feedback_->getPosition();  // start (current value)

    std::stringstream trajectory_ss;  // for debug only

    for (auto i = 0; i < target.size(); ++i) {
        pos(i, 1) = target.at(i) * kDegToRad;  // end (target value)
        if (debug_mode_) {
            trajectory_ss << std::to_string(pos(i, 1)) << "";
        }
    }

    // Create velocity and acceleration constraints
    Eigen::VectorXd max_vel(n_actuators_);
    // TODO: although these are identical to safety.xml, no hard-coding!!
    max_vel << 0.157, 0.157, 0.157, 0.157, 0.079;  // rad/s, ~= 9.0 & 4.5 deg/s

    Eigen::VectorXd max_acc(n_actuators_);
    max_acc.setConstant(0.2);  // rad/s^2, ~= 11.5 deg/s^2 (arbitrary)

    // Use HEBI's trajectory time estimation
    Eigen::VectorXd t_segment =
        hebi::trajectory::Trajectory::estimateSegmentTimesTrapezoidal(pos,
                                                                      max_vel,
                                                                      max_acc);

    Eigen::VectorXd t_waypoint =
        hebi::trajectory::Trajectory::segmentTimesToWaypointTimes(t_segment);

    // TODO: I'd like to compare this with the HEBI-generated times above
    /*
    // Compute trajectory duration for each joint to find the max
    double pos_max_diff = 0;
    for (auto i = 0; i < n_actuators_; i++) {
        pos_max_diff = std::max(abs(pos(i, 1) - pos(i, 0)), pos_max_diff);
    }

    Eigen::VectorXd t_trajectory(2);
    t_trajectory << 0, pos_max_diff / kMaxVel;
    */

    // Log start time and create trajectory
    // NOTE: let QP solver handle vel and accel by setting the parameters
    //       "velocities" and "accelerations" to nullptr
    trajectory_start_time_ = std::chrono::steady_clock::now();
    trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(t_waypoint,
                                                                      pos,
                                                                      nullptr,
                                                                      nullptr);

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
 * @brief Sets the state of automatic dynamic/gravity compensation based on the
 *        loaded HRDF model.
 *
 * @param enabled Whether to compensate for inertia/gravity
 */
void HebiThread::SetModelBasedComp(const bool& enabled) {
    logger_->Debug("HEBI - " + std::string(enabled ? "Enabling" : "Disabling")
                   + " model-based dynamic compensation");

    model_based_comp_en_ = enabled;
}

/**
 * @brief Sets the state of the central joint's torque control.
 *
 * @param enabled Whether to control torque experienced by the central joint
 */
void HebiThread::SetTorqueControl(const bool& enabled) {
    logger_->Debug("HEBI - " + std::string(enabled ? "Enabling" : "Disabling")
                   + " torque-based movement control");

    torque_control_en_ = enabled;

    if (!torque_control_en_) {
        // "Manual" mode: only force allow movement if torque compensation is disabled
        movement_en_ = true;
    }
}

/**
 * @brief Sets the bounds for the central joint's torque control.
 *
 * @param lower_bound Upper torque bound, in Nm
 * @param upper_bound Lower torque bound, in Nm
 */
void HebiThread::SetTorqueCompBounds(const double& lower_bound,
                                     const double& upper_bound) {
    torque_comp_lower_bound_ = lower_bound;
    torque_comp_upper_bound_ = upper_bound;

    logger_->Debug("HEBI - Torque compensation bounds updated: lower="
                   + std::to_string(lower_bound)
                   + " Nm, upper=" + std::to_string(upper_bound) + " Nm");
}
