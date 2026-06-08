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
#include <filesystem>
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
 * !Control Algorithm Settings (slots)
 */

// Improving Readability of Conversions

constexpr double kDegToRad = M_PI / 180;
constexpr double kRadToDeg = 180 / M_PI;

// HEBI Functions

constexpr int32_t kTimeout = 3000;  // ms
constexpr double kMaxVel = 0.052;   // rad/s (~3 deg/s)

// Status Message Formatting

constexpr int kLabelWidth = 20;  // longest label = 19 char, +1 for visuals
constexpr int kValueWidth = 6;  // format is [sign][#,2][.][#,2], +1 for visuals

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

/**
 * @brief Formats value to fixed-precision string.
 *
 * @param value Value to format
 * @param precision Number of decimal places
 */
std::string FormatDouble(double value, int precision = 2) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Delegating constructor. Automatically configures HebiThread based on
 *        `LIBRA_VERSION`.
 *
 * @param parent Owning Qt widget
 * @param debug_mode Whether to output verbose debug text
 */
HebiThread::HebiThread(QObject* parent, const bool& debug_mode)
    : HebiThread(parent, {"LIBRA"}, Actuator::GetHebiDefault(), debug_mode) {}

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
    logger_ = std::make_unique<Ros2Logger>(this->get_logger(), debug_mode_);
#else
    logger_ = std::make_unique<QtLogger>("hebi_manager", debug_mode_);
#endif

    // Initialize HEBI objects
    command_ = std::make_shared<hebi::GroupCommand>(n_actuators_);
    feedback_ = std::make_shared<hebi::GroupFeedback>(n_actuators_);

    // "Translate" string names into corresponding enums for feedback_map
    enum_names_.resize(n_actuators_);
    for (std::size_t i = 0; i < n_actuators_; ++i) {
        enum_names_[i] = Actuator::StringToNameEnum(names_[i]);
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
    target_pub_ = this->create_publisher<msgJointState>("/joint_targets", 10);

    auto hebi_names = Joint::GetHebiStrings();
# if LIBRA_VERSION == 1
    // TODO: temporary solution to publish the static joint to the ROS2 domain
    hebi_names.push_back("Static-Manip");
# endif
    state_msg_.name = hebi_names;
    target_msg_.name = hebi_names;

# if LIBRA_VERSION == 1
    state_msg_.position.resize(n_actuators_ + 1);  // +1 for static joint
    state_msg_.velocity.resize(n_actuators_ + 1);
    state_msg_.effort.resize(n_actuators_ + 1);
    target_msg_.position.resize(n_actuators_ + 1);
    target_msg_.velocity.resize(n_actuators_ + 1);
    target_msg_.effort.resize(n_actuators_ + 1);
# else
    state_msg_.position.resize(n_actuators_);
    state_msg_.velocity.resize(n_actuators_);
    state_msg_.effort.resize(n_actuators_);
    target_msg_.position.resize(n_actuators_);
    target_msg_.velocity.resize(n_actuators_);
    target_msg_.effort.resize(n_actuators_);
# endif
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
 * @brief Converts raw feedback to effective joint values, handling special
 *        cases like differential joints.
 *
 * @param feedback Raw feedback from actuators
 * @param type Type of feedback (e.g., position, velocity, effort)
 */
std::unordered_map<Joint::Name, double> HebiThread::GetJointFeedbackMap(
    const std::shared_ptr<hebi::GroupFeedback>& feedback,
    const Actuator::Feedback& type) {
    std::unordered_map<Joint::Name, double> joint_feedback_map;

    for (int i = 0; i < n_actuators_; ++i) {
        double val = 0;

        // Retrieve value based on feedback type
        if (type == Actuator::Feedback::kTargetPos) {
            val = feedback->getPositionCommand()[i] * kRadToDeg;
        } else if (type == Actuator::Feedback::kTargetVel) {
            val = feedback->getVelocityCommand()[i] * kRadToDeg;
        } else if (type == Actuator::Feedback::kTargetTorque) {
            val = feedback->getEffortCommand()[i];
        } else if (type == Actuator::Feedback::kActualPos) {
            val = feedback->getPosition()[i] * kRadToDeg;
        } else if (type == Actuator::Feedback::kActualVel) {
            val = feedback->getVelocity()[i] * kRadToDeg;
        } else if (type == Actuator::Feedback::kActualTorque) {
            val = feedback->getEffort()[i];
        } else {
            continue;  // unknown type
        }

        // Handle complex joints first

#if LIBRA_VERSION == 1
        // [Complex Joint] MA and MB differential drive -> Roll and Pitch
        double ma_val = 0;  // storage for differential drive calculation
        if (enum_names_[i] == Actuator::Name::kMA) {
            ma_val = val;
            continue;  // MA will be handled with MB, so do nothing
        }
        if (enum_names_[i] == Actuator::Name::kMB) {
            const double mb_val = val;

            joint_feedback_map[Joint::Name::kRoll] =
                Joint::ActuatorToJointDifferential(Joint::Name::kRoll, ma_val,
                                                   mb_val);
            joint_feedback_map[Joint::Name::kPitch] =
                Joint::ActuatorToJointDifferential(Joint::Name::kPitch, ma_val,
                                                   mb_val);

            continue;
        }
#endif
        // Regular 1-DoF joints should be 1-to-1
        auto name = static_cast<Joint::Name>(
            enum_names_[i]);  // TODO: casting Actuator::Name to Joint::Name is
                              //       horribly bug-prone
        joint_feedback_map[name] = Joint::ActuatorToJointSimple(name, val);
    }

#if LIBRA_VERSION == 1
    // Send J3 position for manipulator pitch correction
    if (type == Actuator::Feedback::kActualPos) {
        emit InformPitch(feedback->getPosition()[Actuator::Name::kJ3]
                         * kRadToDeg);
    }
#endif

    return joint_feedback_map;
}

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
    const double arm_torque_theta = (eff[Actuator::Name::kPitch] >= 0) ? 0.0
                                                                       : M_PI;
#endif

    // Apply hysteresis logic
    // TODO: the current "algorithm" is just simple hysteresis which
    //       stops ALL arm movement until the counterweight is full
    //       enough. This is not ideal, and we should be predicting how
    //       arm movement will affect the central joint torque.
    if (arm_torque_r > torque_comp_upper_bound_) {
        // If arm torque is too high, ensure movement is disabled before reporting
        // direction of torque to arduino_thread for fluid system balancing
        if (movement_en_) {
            movement_en_ = false;

            logger_->Debug("ATC - Movement disabled due to high torque ("
                           + FormatDouble(arm_torque_r) + " Nm > "
                           + FormatDouble(torque_comp_upper_bound_) + " Nm)");
        } else {  // movement is disabled
            emit ReportArmTorque(arm_torque_theta);
        }

    } else if (arm_torque_r >= torque_comp_lower_bound_) {
        // While significant arm torque still exists, continue reporting
        // direction of torque to arduino_thread
        emit ReportArmTorque(arm_torque_theta);

    } else {
        // Once arm torque reaches an acceptable level, disable fluid system
        // balancing and re-enable movement
        emit ReportArmTorque(std::nullopt);
        if (!movement_en_) {
            movement_en_ = true;

            logger_->Debug("ATC - Movement re-enabled ("
                           + FormatDouble(arm_torque_r) + " Nm < "
                           + FormatDouble(torque_comp_lower_bound_) + " Nm)");
        }
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

            logger_->Debug("Trajectory complete");

            // Calculate effort commands to counteract gravity
            if (model_based_comp_en_ && model_ != nullptr) {
                // TODO: test this!!
                // NOTE: This may be problematic... I deleted it for a
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
 * @brief Sends actuator feedback to whoever's listening (Qt/ROS2).
 *
 * @note Command buffers are reused to avoid repeated allocations at 100 Hz.
 */
void HebiThread::SendFeedback() {
    // Report important statuses individually
    emit ReportFeedback(GetJointFeedbackMap(feedback_,
                                            Actuator::Feedback::kTargetPos),
                        Actuator::Feedback::kTargetPos);
    emit ReportFeedback(GetJointFeedbackMap(feedback_,
                                            Actuator::Feedback::kActualPos),
                        Actuator::Feedback::kActualPos);
    emit ReportFeedback(GetJointFeedbackMap(feedback_,
                                            Actuator::Feedback::kActualTorque),
                        Actuator::Feedback::kActualTorque);

    // Report miscellaneous statuses in one batch
    emit ReportStatus(GetStatus(), type_);

#ifdef BUILD_WITH_ROS2
    // Make actuator state info available to other ROS2 nodes
    PublishState();
#endif
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
    ss << std::left << std::setw(kLabelWidth) << "Actuator Name";
    for (int i = 0; i < n_actuators_; ++i) {
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
 * @brief Publishes HEBI actuator state(s).
 */
void HebiThread::PublishState() {
    if (!state_pub_ || !target_pub_ || group_ == nullptr) {
        return;
    }

    // Update header timestamps
    const auto stamp = this->now();
    state_msg_.header.stamp = stamp;
    target_msg_.header.stamp = stamp;

    // Get joint-space feedback
    const auto actual_pos_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kActualPos);
    const auto actual_vel_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kActualVel);
    const auto actual_eff_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kActualTorque);

    const auto target_pos_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kTargetPos);
    const auto target_vel_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kTargetVel);
    const auto target_eff_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kTargetTorque);

    // Populate messages and publish
    for (int i = 0; i < n_actuators_; ++i) {
        const auto joint = static_cast<Joint::Name>(i);

        const double actual_pos = actual_pos_map.at(joint);
        const double actual_vel = actual_vel_map.at(joint);
        const double actual_eff = actual_eff_map.at(joint);

        state_msg_.position[i] = actual_pos * kDegToRad;
        state_msg_.velocity[i] = actual_vel * kDegToRad;
        state_msg_.effort[i] = actual_eff;

        const double target_pos = target_pos_map.at(joint);
        const double target_vel = target_vel_map.at(joint);
        const double target_eff = target_eff_map.at(joint);

        target_msg_.position[i] = target_pos * kDegToRad;
        target_msg_.velocity[i] = target_vel * kDegToRad;
        target_msg_.effort[i] = target_eff;
    }

# if LIBRA_VERSION == 1
    // TODO: temporary static manipulator joint
    state_msg_.position[n_actuators_] = 0.0;
    state_msg_.velocity[n_actuators_] = 0.0;
    state_msg_.effort[n_actuators_] = 0.0;

    target_msg_.position[n_actuators_] = 0.0;
    target_msg_.velocity[n_actuators_] = 0.0;
    target_msg_.effort[n_actuators_] = 0.0;
# endif

    state_pub_->publish(state_msg_);
    target_pub_->publish(target_msg_);
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
 * @see CheckTorqueControl(), ExecuteMovement(), SendFeedback()
 */
void HebiThread::run() {
    logger_->Debug("Starting thread");

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
        SendFeedback();

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

        logger_->Debug("Found the following actuators (Family|Name):");
        for (auto entry : *entry_list) {
            logger_->Debug("  " + entry.family_ + " | " + entry.name_);
        }
    }

    // Filter lookup for relevant actuator(s)
    // NOTE: Don't save to class member `group_` until checks have passed
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

    logger_->Debug("Connection successful");
    emit Connected(true);

    // Load robot kinematics
    // TODO: this doesn't seem to help at all since the HRDF format is too
    //       limited to model the LIBRA-I's differential joint. However, it may
    //       be useful for LIBRA-II.
    /*
    model_ = hebi::robot_model::RobotModel::loadHRDF(
        "./bin/shared/hebi/libra.hrdf");
    if (model_ == nullptr) {
        emit ErrorThrown("HEBI - Failed to load HRDF!");
        return;
    }
    model_->getMasses(model_masses_);
    */

    // Command actuator(s) to hold current position
    group_ = group;
    group_->getNextFeedback(*feedback_);
    command_->setPosition(feedback_->getPosition());
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
        StopHebiLog();

        // Destructing hebi::Group automatically cleans it up
        group_.reset();

        logger_->Debug("Gracefully disconnected from actuator(s)");
        emit Connected(false);
    }
}

/**
 * @brief Sets movement targets/trajectories for all connected actuators.
 *
 * @param target Target angles (absolute) for all actuators, in degrees
 *
 * @note The `target` parameter should be in "actuator space" rather than "joint
 *       space" to keep `HebiThread` decoupled from the kinematics of specific
 *       robot designs. For example, in LIBRA-I, "actuator space" corresponds to
 *       the individual actuators in the 2-DoF differential drive (MA and MB)
 *       and 3-DoF arm (J1, J2, J3), while "joint space" corresponds to the
 *       effective Roll, Pitch, J1 (Yaw), J2 (Yaw), and J3 (Pitch) angles.
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

    /* NOTE: The following trajectory generation method is theoretically better
             (it was added in a newer version of the HEBI API), but in practice
             it results in very unstable oscillations. Therefore, a simpler
             method is used below.

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
    max_vel << 0.052, 0.052, 0.052, 0.052, 0.018;  // rad/s, ~= 3 & 1 deg/s

    Eigen::VectorXd max_acc(n_actuators_);
    max_acc.setConstant(0.2);  // rad/s^2, ~= 11.5 deg/s^2 (arbitrary)

    // Use HEBI's trajectory time estimation
    Eigen::VectorXd t_segment =
        hebi::trajectory::Trajectory::estimateSegmentTimesTrapezoidal(pos,
                                                                      max_vel,
                                                                      max_acc);

    Eigen::VectorXd t_waypoint =
        hebi::trajectory::Trajectory::segmentTimesToWaypointTimes(t_segment);

    // Log start time and create trajectory
    // NOTE: Let QP solver handle vel and accel by setting the parameters
    //       "velocities" and "accelerations" to nullptr
    trajectory_start_time_ = std::chrono::steady_clock::now();
    trajectory_ =
        hebi::trajectory::Trajectory::createUnconstrainedQp(t_waypoint, pos,
                                                            nullptr, nullptr);
    */

    // Make position, velocity, and acceleration commands for start & end points
    Eigen::MatrixXd pos(n_actuators_, 2);
    // Eigen::MatrixXd vel = Eigen::MatrixXd::Constant(num_actuators_, 2, kMaxVel);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(n_actuators_, 2);    // default
    Eigen::MatrixXd accel = Eigen::MatrixXd::Zero(n_actuators_, 2);  // default

    std::stringstream trajectory_ss;  // for debug only

    // Populate positions
    group_->getNextFeedback(*feedback_);
    pos.col(0) = feedback_->getPosition();  // start (current value)

    for (int i = 0; i < target.size(); ++i) {
        // NOTE: Targets assumed to already be in "actuator space"
        pos(i, 1) = target.at(i) * kDegToRad;

        if (debug_mode_) {
            trajectory_ss << std::to_string(pos(i, 1));
        }
    }

    // Determine greatest change in position for calculating trajectory times
    double max_difference = 0;
    for (int i = 0; i < n_actuators_; ++i) {
        max_difference = std::max(abs(pos(i, 1) - pos(i, 0)), max_difference);
    }

    // Calculate trajectory start and end times
    Eigen::VectorXd time(2);
    time << 0, max_difference / kMaxVel;

    // Log start time and create trajectory
    trajectory_start_time_ = std::chrono::steady_clock::now();
    trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(time, pos,
                                                                      &vel,
                                                                      &accel);

    logger_->Debug("Set trajectory target(s) to " + trajectory_ss.str()
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

    logger_->Debug("Trajectory reset");
}

/**
 * @brief Loads new PID gains for the connected actuators.
 *
 * @param file_path Path to an XML gains file
 */
void HebiThread::LoadGains(const QString& file_path) {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't load gains; not connected!");
        return;
    }

    hebi::GroupCommand gain_cmd(group_->size());
    if (!gain_cmd.readGains(file_path.toStdString())) {
        emit ErrorThrown("HEBI - Failed to load gains from: " + file_path);
        return;
    }

    if (!group_->sendCommandWithAcknowledgement(gain_cmd, kTimeout)) {
        emit ErrorThrown(
            "HEBI - Didn't receive acknowledgement from gains update!");
        return;
    }

    logger_->Info("Successfully loaded gains from: " + file_path.toStdString());
}

/**
 * @brief Starts HEBI logging.
 *
 * @note Hebilog file names default to the following format:
 *       `log_file_YYYY-MM-DD_HH-MM-SS.MSS.hebilog`
 */
void HebiThread::StartHebiLog() {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't start logging; not connected!");
        return;
    }

    if (logging_active_) {
        logger_->Warn("Logging is already active");
        return;
    }

    // Ensure a log directory exists (or else group_->startLog() will fail!)
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
        emit ErrorThrown("HEBI - Failed to start logging! Ensure log directory "
                         "(log/) exists in CWD.");
        return;
    }
    logging_active_ = true;

    auto full_path = std::filesystem::canonical(log_path).string();
    logger_->Info("Started logging to: " + full_path);
}

/**
 * @brief Stops HEBI logging.
 */
void HebiThread::StopHebiLog() {
    if (group_ == nullptr) {
        logger_->Warn("Can't stop logging; not connected");
        return;
    }

    if (!logging_active_) {
        logger_->Warn("Logging is not currently active");
        return;
    }

    // Stop logging
    group_->stopLog();
    logging_active_ = false;

    logger_->Info("Stopped logging");
}

//------------------------------------------------------------------------------
// !Control Algorithm Settings (slots)
//------------------------------------------------------------------------------

/**
 * @brief Sets the state of automatic dynamic/gravity compensation based on the
 *        loaded HRDF model.
 *
 * @param enabled Whether to compensate for inertia/gravity
 */
void HebiThread::EnableModelBasedComp(const bool& enabled) {
    logger_->Debug(std::string(enabled ? "Enabling" : "Disabling")
                   + " model-based dynamic compensation");

    model_based_comp_en_ = enabled;
}

/**
 * @brief Sets the state of the central joint's torque control.
 *
 * @param enabled Whether to control torque experienced by the central joint
 */
void HebiThread::EnableTorqueControl(const bool& enabled) {
    torque_control_en_ = enabled;

    if (!torque_control_en_) {
        // "Manual" mode: only force allow movement if torque compensation is disabled
        movement_en_ = true;
    }

    logger_->Debug("ATC - Automatic torque compensation "
                   + std::string(enabled ? "enabled" : "disabled"));
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

    logger_->Debug("ATC - Torque compensation bounds updated: lower="
                   + FormatDouble(lower_bound)
                   + " Nm, upper=" + FormatDouble(upper_bound) + " Nm");
}
