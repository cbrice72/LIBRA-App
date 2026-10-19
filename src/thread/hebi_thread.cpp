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
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
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
 * !Calibration (slots)
 */

// Improving Readability of Conversions

constexpr double kDegToRad = M_PI / 180;
constexpr double kRadToDeg = 180 / M_PI;

// HEBI Functions

constexpr int32_t kTimeout = 3000;  // ms
constexpr double kMaxVel = 0.052;   // rad/s (~3 deg/s)

#if LIBRA_VERSION == 2
// Motion Controller Logging

// TODO: guess; how often the debug log shows the model's values
constexpr double kDiagnosticsPeriod = 3.0;  // s
#endif

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
std::string FormatDouble(const double& value, const int& precision = 2) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

/**
 * @brief Maps an actuator to the joint it drives (simple joints only).
 *
 * @param actuator Named enum value of the actuator
 *
 * @note Returns `Joint::Name::kUndefined` for actuators that are part of
 *       complex joints (LIBRA-I's differential 2-DoF joint w/ MA and MB).
 */
Joint::Name JointFromActuator(const Actuator::Name& actuator) {
    switch (actuator) {
#if LIBRA_VERSION == 1
        case Actuator::Name::kJ1:
            return Joint::Name::kJ1;
        case Actuator::Name::kJ2:
            return Joint::Name::kJ2;
        case Actuator::Name::kJ3:
            return Joint::Name::kJ3;
#elif LIBRA_VERSION == 2
        case Actuator::Name::kPitch:
            return Joint::Name::kPitch;
        case Actuator::Name::kYaw:
            return Joint::Name::kYaw;
#endif
        default:
            return Joint::Name::kUndefined;
    }
}

#if LIBRA_VERSION == 2
/**
 * @brief Formats an optional value ("n/a" if empty).
 *
 * @param value Value to format
 * @param precision Decimal places
 */
std::string FormatOptional(const std::optional<double>& value,
                           const int& precision = 2) {
    return value ? FormatDouble(*value, precision) : "n/a";
}

/**
 * @brief Encodes a CW command for `ReportArmTorque`.
 *
 * @param command What the CW should do
 *
 * @note This is the torque-direction encoding LIBRA-I uses;
 *       `WaterController::UpdateTorqueFeedback` only checks "0 / non-zero /
 *       none".
 */
std::optional<double> CwCommandToReport(const Motion::CwCommand& command) {
    switch (command) {
        case Motion::CwCommand::kFill:
            return M_PI;
        case Motion::CwCommand::kDrain:
            return 0.0;
        default:
            return std::nullopt;
    }
}
#endif

}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Delegating constructor. Automatically configures HebiThread based on
 *        `LIBRA_VERSION`.
 *
 * @param parent Owning Qt widget
 * @param motion_controller (LIBRA-II) Custom kinematics-informed motion controller
 * @param debug_mode Whether to output verbose debug text
 */
HebiThread::HebiThread(QObject* parent,
#if LIBRA_VERSION == 2
                       std::shared_ptr<Libra2MotionController> motion_controller,
#endif
                       const bool& debug_mode)
    : HebiThread(parent,
#if LIBRA_VERSION == 2
                 std::move(motion_controller),
#endif
                 {"LIBRA"}, Actuator::GetHebiDefault(), debug_mode) {
}

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param motion_controller (LIBRA-II) Custom kinematics-informed motion controller
 * @param families Families of actuators to search for names in
 * @param names Names of actuators to connect to
 * @param debug_mode Whether to output verbose debug text
 */
HebiThread::HebiThread(QObject* parent,
#if LIBRA_VERSION == 2
                       std::shared_ptr<Libra2MotionController> motion_controller,
#endif
                       std::vector<std::string> families,
                       std::vector<std::string> names, const bool& debug_mode)
    : AbstractActuatorThread(parent, debug_mode, Actuator::Type::kHebi),
      families_(std::move(families)),
      names_(std::move(names)),
      group_(nullptr),
      n_actuators_(names_.size()),
      loop_period_ms_(10)  // 100 Hz
#if LIBRA_VERSION == 2
      ,
      motion_controller_(std::move(motion_controller))
#endif
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
    logger_->Debug("Initializing HebiThread");

    // Initialize HEBI objects
    command_ = std::make_shared<hebi::GroupCommand>(n_actuators_);
    feedback_ = std::make_shared<hebi::GroupFeedback>(n_actuators_);

    // "Translate" string names into corresponding enums for feedback_map
    enum_names_.resize(n_actuators_);
    for (std::size_t i = 0; i < n_actuators_; ++i) {
        enum_names_[i] = Actuator::StringToNameEnum(names_[i]);

        // Every actuator must drive a known joint
#if LIBRA_VERSION == 1
        const bool differential = (enum_names_[i] == Actuator::Name::kMA)
                                  || (enum_names_[i] == Actuator::Name::kMB);
#else
        const bool differential = false;
#endif
        if (!differential
            && JointFromActuator(enum_names_[i]) == Joint::Name::kUndefined) {
            logger_->Error("Actuator " + names_[i]
                           + " has no matching joint; its feedback and "
                             "commands will be ignored");
        }
    }

    // Resize vectors to match number of actuators
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
    // NOTE: this doesn't belong here, but until we replace either of LIBRA-I's
    //       or LIBRA-II's static joint with something else, this is the easiest
    //       way to get it into the ROS2 message.
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

    double ma_val = 0;  // storage for differential drive calculation

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
        const auto joint = JointFromActuator(enum_names_[i]);
        if (joint == Joint::Name::kUndefined) {
            continue;  // user is only warned once (in constructor)
        }
        joint_feedback_map[joint] = Joint::ActuatorToJointSimple(joint, val);
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

#if LIBRA_VERSION == 1
/**
 * @brief Checks torque on the central joint (LIBRA-I's 2-DoF joint) and
 *        enforces movement limits.
 */
void HebiThread::CheckTorqueControl() {
    if (!torque_control_en_) {
        return;
    }

    // Retrieve torque magnitude and direction (i.e., polar coords)
    const auto& eff = feedback_->getEffort();

    const double arm_roll_torque =
        Joint::ActuatorToJointDifferential(Joint::Name::kRoll,
                                           eff[Actuator::Name::kMA],
                                           eff[Actuator::Name::kMB]);
    const double arm_pitch_torque =
        Joint::ActuatorToJointDifferential(Joint::Name::kPitch,
                                           eff[Actuator::Name::kMA],
                                           eff[Actuator::Name::kMB]);
    const double arm_torque_r =
        std::hypot(arm_roll_torque,    // hypot calculates sqrt(x^2 + y^2)
                   arm_pitch_torque);  // without overflow/underflow issues

    const double arm_torque_theta =
        std::atan2(-eff[Actuator::Name::kMA]
                       + eff[Actuator::Name::kMB],  // pitch
                   -eff[Actuator::Name::kMA]
                       - eff[Actuator::Name::kMB]);  // roll

    // Apply 3-tier hysteresis logic
    // TODO: Ideally we should be predicting how a trajectory will affect the
    //       central joint's feedback torque, maybe with ROS2 or something.
    if (arm_torque_r > torque_comp_upper_bound_) {
        // If arm torque is too high, disable movement before reporting the
        // torque direction to WaterController for CW balancing
        if (movement_en_) {
            movement_en_ = false;

            logger_->Info("Movement disabled due to high torque ("
                          + FormatDouble(arm_torque_r) + " Nm > "
                          + FormatDouble(torque_comp_upper_bound_) + " Nm)");
        } else {  // movement is disabled
            emit ReportArmTorque(arm_torque_theta);
        }

    } else if (arm_torque_r >= torque_comp_lower_bound_) {
        // While significant arm torque remains, keep reporting its direction
        // to WaterController
        emit ReportArmTorque(arm_torque_theta);
        playback_rate_ =
            kTorqueCompSlowRate;  // avoids sudden stop if upper bound is hit

    } else {
        // Once arm torque reaches an acceptable level, disable fluid system
        // balancing and re-enable movement
        emit ReportArmTorque(std::nullopt);
        playback_rate_ = 1.0;  // full speed
        if (!movement_en_) {
            movement_en_ = true;

            logger_->Info("Movement re-enabled (" + FormatDouble(arm_torque_r)
                          + " Nm < " + FormatDouble(torque_comp_lower_bound_)
                          + " Nm)");
        }
    }
}

/**
 * @brief Executes trajectory following or position hold with dynamic/gravity
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
bool HebiThread::ExecuteMovement(std::chrono::duration<double> dt,
                                 Eigen::VectorXd& cmd_pos,
                                 Eigen::VectorXd& cmd_vel,
                                 Eigen::VectorXd& cmd_acc,
                                 Eigen::VectorXd& cmd_eff) {
    // Playback rate from ATC hysteresis (0 = paused)
    const double rate = movement_en_ ? playback_rate_ : 0.0;

    std::unique_lock<std::mutex> lock(trajectory_mutex_);

    // Determine movement command
    if (trajectory_ != nullptr) {
        t_trajectory_s_ += dt.count() * rate;

        if (t_trajectory_s_ < trajectory_->getDuration()) {
            // --- MODE 1: TRAJECTORY FOLLOWING ---

            // Build next step of trajectory
            // TODO: This used to pass clock ticks (i.e., nanoseconds), but
            //       apparently HEBI expects seconds. Need to verify this.
            trajectory_->getState(t_trajectory_s_, &cmd_pos, &cmd_vel, &cmd_acc);
            command_->setPosition(cmd_pos);
            command_->setVelocity(cmd_vel * rate);
        } else {
            // --- MODE 2: POSITION HOLDING ---

            // Trajectory is complete
            trajectory_.reset();

            logger_->Info("Trajectory complete");
        }
    }
    lock.unlock();

    // Send movement command
    return group_->sendCommand(*command_);
}
#endif  // LIBRA_VERSION == 1

/**
 * @brief Starts a rest-to-rest trajectory between two poses.
 *
 * @param start_pos Start pose (rad, actuator space)
 * @param target_pos End pose (rad, actuator space)
 *
 * @note LIBRA-I calls this from the `MainWindow` thread (via `SetTarget()`).
 *       LIBRA-II calls this from the motion control loop (via `move_to`).
 */
void HebiThread::StartTrajectory(const Eigen::VectorXd& start_pos,
                                 const Eigen::VectorXd& target_pos) {
    // TODO: remove this warning once the new trajectory timing has been
    //       verified on hardware
    if (!time_unit_warned_) {
        time_unit_warned_ = true;
        logger_->Warn("Trajectory time is now passed to HEBI in seconds (it "
                      "used to be clock ticks, i.e., nanoseconds); verify that "
                      "moves follow the expected speed");
    }

    // Make position, velocity, and acceleration commands for start & end points
    Eigen::MatrixXd pos(n_actuators_, 2);
    // Eigen::MatrixXd vel = Eigen::MatrixXd::Constant(num_actuators_, 2, kMaxVel);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(n_actuators_, 2);    // default
    Eigen::MatrixXd accel = Eigen::MatrixXd::Zero(n_actuators_, 2);  // default

    std::stringstream trajectory_ss;  // for debug only

    // Populate positions (assumed to be in "actuator space")
    pos.col(0) = start_pos;
    pos.col(1) = target_pos;

    if (debug_mode_) {
        trajectory_ss << std::fixed << std::setprecision(2)
                      << pos.col(1).format(matrix_print_format_);
    }

    // Determine greatest change in position for calculating trajectory times
    double max_difference = 0;
    for (int i = 0; i < n_actuators_; ++i) {
        max_difference = std::max(std::abs(pos(i, 1) - pos(i, 0)),
                                  max_difference);
    }

    // Calculate trajectory start and end times
    Eigen::VectorXd time(2);
#if LIBRA_VERSION == 2
    // A zero-length move would give a zero-duration (invalid) trajectory, so
    // allow at least one loop period (the shortest the loop can follow)
    time << 0, std::max(max_difference / kMaxVel, loop_period_ms_ / 1000.0);
#else
    time << 0, max_difference / kMaxVel;
#endif

    // Create trajectory and log start time
    auto trajectory =
        hebi::trajectory::Trajectory::createUnconstrainedQp(time, pos, &vel,
                                                            &accel);
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        t_trajectory_s_ = 0.0;
        trajectory_ = std::move(trajectory);
    }

    logger_->Debug("Set trajectory target(s) to " + trajectory_ss.str()
                   + " rad");
}

#if LIBRA_VERSION == 2
/**
 * @brief Tells the UI when movement starts/stops, when a calibration fails,
 *        and when the arm model appears or goes away.
 *
 * @note "Moving" covers trajectory-based motion, the CW alone moving the arm, a
 *       queued move, and a running calibration; the UI disables mode switching
 *       while it is true.
 */
void HebiThread::ReportMotionStatus() {
    const auto status = motion_controller_->GetCalibrationStatus();

    // Failure pops up once (success and cancellation are already logged)
    if (calibration_running_ && !Motion::IsCalibrationRunning(status)) {
        calibration_running_ = false;
        if (status == Motion::CalibrationStatus::kFailed) {
            emit ErrorThrown("HEBI - "
                             + QString::fromStdString(
                                 motion_controller_->GetCalibrationMessage()));
        }
    }

    const bool calibrated = motion_controller_->IsCalibrated();
    if (arm_calibrated_.exchange(calibrated) != calibrated) {
        emit ArmModelChanged(calibrated);
    }

    const bool moving = calibration_running_ || motion_controller_->IsMoving();
    if (moving != moving_) {
        moving_ = moving;
        emit MovementStateChanged(moving);
    }
}

/**
 * @brief LIBRA-II control cycle: hands measurements to the motion controller,
 *        applies what it returns, and sends the command to the actuators.
 *
 * @param dt Time since last control cycle
 * @param cmd_pos Trajectory position buffer (rad, actuator space)
 * @param cmd_vel Velocity reference buffer (rad/s, actuator space); holds
 *                PREVIOUS cycle's reference on entry
 * @param cmd_acc Acceleration reference buffer (rad/s², actuator space); same
 * @param cmd_eff Effort command buffer (Nm, actuator space)
 *
 * @note Motion controller only sees joint space; conversion to and from
 *       actuator space happens here. Command buffers are reused to avoid
 *       repeated allocations at 100 Hz.
 */
bool HebiThread::ExecuteMotionController(std::chrono::duration<double> dt,
                                         Eigen::VectorXd& cmd_pos,
                                         Eigen::VectorXd& cmd_vel,
                                         Eigen::VectorXd& cmd_acc,
                                         Eigen::VectorXd& cmd_eff) {
    constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    const auto n = static_cast<std::size_t>(n_actuators_);

    // Connect() normally sets these; guard against a first cycle that beats it
    if (pos_cmd_.size() != n_actuators_ || vel_cmd_.size() != n_actuators_) {
        pos_cmd_ = feedback_->getPosition();
        vel_cmd_ = Eigen::VectorXd::Zero(n_actuators_);
    }

    // Free mode: send nothing to the pitch joint
    if (free_mode_) {
        pos_cmd_.setConstant(kNone);
        vel_cmd_.setZero();
        emit ReportArmTorque(std::nullopt);
        ReportMotionStatus();
        return true;  // nothing to send
    }

    // No trajectory, no reference (e.g., after Stop())
    bool trajectory_active = false;
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_active = (trajectory_ != nullptr);
    }
    if (!trajectory_active) {
        cmd_vel.setZero();
        cmd_acc.setZero();
    }

    const double dt_s = dt.count();

    // Measurements and last cycle's reference, converted to joint space
    Motion::MotionState state;
    state.dt = dt_s;
    state.pos.resize(n);
    state.vel.resize(n);
    state.torque_actuator.resize(n);
    state.ref_vel.resize(n);
    state.ref_acc.resize(n);
    for (int i = 0; i < n_actuators_; ++i) {
        const auto joint = JointFromActuator(enum_names_[i]);

        state.pos[i] = Joint::ActuatorToJointSimple(joint,
                                                    feedback_->getPosition()[i]);
        state.vel[i] = Joint::ActuatorToJointSimple(joint,
                                                    feedback_->getVelocity()[i]);
        state.torque_actuator[i] =
            Joint::ActuatorToJointSimple(joint, feedback_->getEffort()[i]);
        state.ref_vel[i] = Joint::ActuatorToJointSimple(joint, cmd_vel[i]);
        state.ref_acc[i] = Joint::ActuatorToJointSimple(joint, cmd_acc[i]);
    }
    state.trajectory_active = trajectory_active;

    const Motion::Libra2Command cmd = motion_controller_->Update(state);
    if (cmd.mode.size() != n) {
        // Motion controller already logged the problem; keep last command
        return group_->sendCommand(*command_);
    }

    // Pass CW command on to WaterController
    if (cmd.cw_command) {
        emit ReportArmTorque(CwCommandToReport(*cmd.cw_command));
    }

    // Stop requested by motion controller (friction sweep ended early)
    if (cmd.stop_trajectory) {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_.reset();
        t_trajectory_s_ = 0.0;
        logger_->Info("Trajectory stopped by motion controller");
    }

    // Move requested by motion controller (calibration, CW-primary, tandem CW-actuator)
    if (cmd.move_to && cmd.move_to->size() == n) {
        Eigen::VectorXd move_pos(n_actuators_);
        for (int i = 0; i < n_actuators_; ++i) {
            move_pos(i) = Actuator::JointToActuatorSimple(enum_names_[i],
                                                          (*cmd.move_to)[i]);
        }
        StartTrajectory(feedback_->getPosition(), move_pos);
    }

    // Sweep requested by motion controller (replaces any move above)
    if (cmd.sweep) {
        StartSweep(*cmd.sweep);
    }

    bool arm_free = false;
    for (int i = 0; i < n_actuators_; ++i) {
        arm_free = arm_free
                   || (cmd.mode[i] == Motion::CommandMode::kEffortOnly);
    }

    // Trajectory following at motion controller's playback rate, or holding
    // (`trajectory_` re-checked under the lock, since `Stop()` may have run)
    const double rate = std::clamp(cmd.playback_rate, 0.0, 1.0);
    std::unique_lock<std::mutex> lock(trajectory_mutex_);
    if (arm_free) {
        // CW moves the arm
        // (reset previous trajectory since hand-off will request a new one later)
        trajectory_.reset();
        t_trajectory_s_ = 0.0;
        cmd_vel.setZero();
        cmd_acc.setZero();
        vel_cmd_.setZero();
    } else if (trajectory_ != nullptr) {
        // --- MODE 1: TRAJECTORY FOLLOWING ---

        // Clock stops while playback is paused and resumes where it was
        t_trajectory_s_ += dt_s * rate;

        if (t_trajectory_s_ < trajectory_->getDuration()) {
            trajectory_->getState(t_trajectory_s_, &cmd_pos, &cmd_vel, &cmd_acc);

            // Playing at `rate` scales velocity by rate and acceleration by
            // rate^2, keeping the reference consistent with position
            cmd_vel *= rate;
            cmd_acc *= rate * rate;
            vel_cmd_ = cmd_vel;
        } else {
            // Trajectory complete: hold exact final pose
            trajectory_->getState(trajectory_->getDuration(), &cmd_pos,
                                  &cmd_vel, &cmd_acc);
            cmd_vel.setZero();
            cmd_acc.setZero();
            vel_cmd_.setZero();
            trajectory_.reset();

            logger_->Info("Trajectory complete");
        }
        pos_cmd_ = cmd_pos;
    } else {
        // --- MODE 2: POSITION HOLDING ---

        vel_cmd_.setZero();
    }
    lock.unlock();

    // Per-joint command (NaN = no command)
    // TODO: not confirmed for Strategy 4 with NaN position/velocity/effort;
    //       verify on hardware (support the arm the first time)
    for (int i = 0; i < n_actuators_; ++i) {
        if (cmd.mode[i] == Motion::CommandMode::kEffortOnly) {
            pos_cmd_(i) = kNone;
            vel_cmd_(i) = kNone;
        } else if (std::isnan(pos_cmd_(i))) {
            // Just left effort-only mode: hold current pose
            pos_cmd_(i) = feedback_->getPosition()[i];
        }

        // Effort feedforward
        const auto& torque = cmd.torque_actuator_cmd[i];
        cmd_eff(i) = torque ? Actuator::JointToActuatorSimple(enum_names_[i],
                                                              *torque)
                            : kNone;
    }
    command_->setPosition(pos_cmd_);
    command_->setVelocity(vel_cmd_);
    command_->setEffort(cmd_eff);

    // Periodic model log: motion controller's model vs. measurements
    t_diag_log_elapsed_s_ += dt_s;
    if (t_diag_log_elapsed_s_ >= kDiagnosticsPeriod) {
        t_diag_log_elapsed_s_ = 0.0;

        const auto& d = cmd.diagnostics;
        logger_->Debug("Model - m_cw=" + FormatOptional(d.cw_mass)
                       + " kg, tau_ext,g=" + FormatOptional(d.torque_ext_g)
                       + " Nm, m_cw,bal(target)=" + FormatOptional(d.cw_mass_tgt)
                       + " kg, delta_m=" + FormatOptional(d.cw_mass_change)
                       + " kg, tau_ext,g(target)="
                       + FormatOptional(d.torque_ext_g_tgt) + " Nm");
    }

    ReportMotionStatus();
    return group_->sendCommand(*command_);
}

/**
 * @brief Whether a calibration may start.
 */
bool HebiThread::CanCalibrate() {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't calibrate; not connected!");
        return false;
    }
    if (free_mode_) {
        emit ErrorThrown("HEBI - Can't calibrate in free mode!");
        return false;
    }
    return true;
}

/**
 * @brief Starts a pitch sweep: rest, ramp up to constant speed, hold it, ramp
 *        down to rest (other joints hold).
 *
 * @param sweep Sweep to follow (joint space)
 *
 * @note Starts from the measured pose rather than `sweep.from`, so there's no
 *       jump. With position, speed, and zero acceleration fixed at each
 *       waypoint, the middle segment is exactly constant speed, and each ramp
 *       (speed v over distance d) takes 2·d / v.
 *
 * @note A rejected sweep cancels the calibration that requested it.
 */
void HebiThread::StartSweep(const Motion::SweepRequest& sweep) {
    const double dir = (sweep.to > sweep.from) ? 1.0 : -1.0;
    const double ramp_end = sweep.from + dir * sweep.ramp_dist;
    const double ramp_start = sweep.to - dir * sweep.ramp_dist;
    const auto& start = feedback_->getPosition();

    // Pitch must start before the first ramp ends, at a speed within limits
    double pitch_start = sweep.from;
    for (int i = 0; i < n_actuators_; ++i) {
        if (JointFromActuator(enum_names_[i]) == Joint::Name::kPitch) {
            pitch_start = Joint::ActuatorToJointSimple(Joint::Name::kPitch,
                                                       start[i]);
        }
    }
    if (!(sweep.speed > 0.0) || sweep.speed > kMaxVel
        || !(sweep.ramp_dist > 0.0) || dir * (ramp_start - ramp_end) < 0.0
        || dir * (ramp_end - pitch_start) <= 0.0) {
        logger_->Error("Rejected sweep: speed=" + FormatDouble(sweep.speed, 4)
                       + " rad/s (limit " + FormatDouble(kMaxVel, 4)
                       + "), from " + FormatDouble(pitch_start, 3) + " to "
                       + FormatDouble(sweep.to, 3) + " rad");
        motion_controller_->CancelCalibration();
        emit ErrorThrown("HEBI - Friction sweep rejected (see the log for the "
                         "reason)!");
        return;
    }

    // Waypoints (joint space): rest, constant speed starts, ends, rest
    const std::array<double, 4> pitch_pos = {pitch_start, ramp_end, ramp_start,
                                             sweep.to};
    const std::array<double, 4> pitch_vel = {0.0, dir * sweep.speed,
                                             dir * sweep.speed, 0.0};
    const double t_ramp_s = 2.0 * sweep.ramp_dist / sweep.speed;
    const double t_const_s = std::abs(ramp_start - ramp_end) / sweep.speed;

    Eigen::VectorXd time(4);
    time << 0, t_ramp_s, t_ramp_s + t_const_s, 2.0 * t_ramp_s + t_const_s;
    Eigen::MatrixXd pos(n_actuators_, 4);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(n_actuators_, 4);
    Eigen::MatrixXd accel = Eigen::MatrixXd::Zero(n_actuators_, 4);
    for (int i = 0; i < n_actuators_; ++i) {
        if (JointFromActuator(enum_names_[i]) != Joint::Name::kPitch) {
            pos.row(i).setConstant(start[i]);  // hold
            continue;
        }
        for (int k = 0; k < 4; ++k) {
            pos(i, k) = Actuator::JointToActuatorSimple(enum_names_[i],
                                                        pitch_pos[k]);
            vel(i, k) = Actuator::JointToActuatorSimple(enum_names_[i],
                                                        pitch_vel[k]);
        }
    }

    auto trajectory =
        hebi::trajectory::Trajectory::createUnconstrainedQp(time, pos, &vel,
                                                            &accel);
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        t_trajectory_s_ = 0.0;
        trajectory_ = std::move(trajectory);
    }

    logger_->Debug("Started sweep to " + FormatDouble(sweep.to, 3) + " rad at "
                   + FormatDouble(sweep.speed, 4) + " rad/s");
}
#endif  // LIBRA_VERSION == 2

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
 * @brief Minor status info for all connected actuators, as comma-delimited
 *        messages (target/actual position and actual torque are sent
 *        separately as signals).
 *
 * @note See the HEBI C++ API page for the full list of available feedback:
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
 * @see CheckTorqueControl ExecuteMovement ExecuteMotionController SendFeedback
 */
void HebiThread::run() {
    logger_->Debug("Starting thread");

    // Initialize buffers for loop efficiency
    Eigen::VectorXd cmd_pos = Eigen::VectorXd::Zero(n_actuators_);
    Eigen::VectorXd cmd_vel = Eigen::VectorXd::Zero(n_actuators_);
    Eigen::VectorXd cmd_acc = Eigen::VectorXd::Zero(n_actuators_);  // <- traj
    Eigen::VectorXd cmd_eff = Eigen::VectorXd::Zero(n_actuators_);  // -> cmd

    // For dynamic compensation (DynamicComp)
    auto last_loop_time = std::chrono::steady_clock::now();

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        if (group_ == nullptr) {
            emit ReportStatus(GetStatus(), type_);  // "Not Connected"
            // Keep the first `dt` after `Connect()` small
            last_loop_time = std::chrono::steady_clock::now();
            QThread::msleep(loop_period_ms_);
            continue;
        }

        // Calculate time since last loop (only used in DynamicComp)
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - last_loop_time;
        last_loop_time = now;

        // Update feedback object
        if (!group_->getNextFeedback(*feedback_)) {
            emit ErrorThrown("HEBI - Failed to receive actuator feedback; "
                             "connection lost!");
            Disconnect();
            continue;
        }

        // Execute control logic
#if LIBRA_VERSION == 2
        const bool command_sent = ExecuteMotionController(dt, cmd_pos, cmd_vel,
                                                          cmd_acc, cmd_eff);
#else
        CheckTorqueControl();
        const bool command_sent = ExecuteMovement(dt, cmd_pos, cmd_vel, cmd_acc,
                                                  cmd_eff);
#endif
        if (!command_sent) {
            emit ErrorThrown("HEBI - Failed to send actuator command; "
                             "connection lost!");
            Disconnect();
            continue;
        }
        SendFeedback();

        // Don't overwhelm network
        QThread::msleep(loop_period_ms_);
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

    // Command actuator(s) to hold current position
    group_ = group;
    if (!group_->getNextFeedback(*feedback_)) {
        emit ErrorThrown("HEBI - Failed to retrieve starting position!");
        group_.reset();
        return;
    }
    command_->setPosition(feedback_->getPosition());

#if LIBRA_VERSION == 2
    // Same pose for loop's position hold; reset motion controller and clock
    pos_cmd_ = feedback_->getPosition();
    vel_cmd_ = Eigen::VectorXd::Zero(n_actuators_);
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        t_trajectory_s_ = 0.0;
    }
    motion_controller_->Reset();
    if (motion_controller_->GetNumJoints()
        != static_cast<std::size_t>(n_actuators_)) {
        emit ErrorThrown("HEBI - Motion controller joint count doesn't match "
                         "the number of actuators; its commands will be "
                         "ignored!");
    }
#endif

    logger_->Info("Connection successful");
    emit Connected(true);
}

/**
 * @brief Terminates the active connection(s).
 */
void HebiThread::Disconnect() {
    // In case this was called in the middle of a movement, gracefully stop
    Stop();

    if (group_ != nullptr) {
        StopHebiLog();

        // Idle CW just in case
        emit ReportArmTorque(std::nullopt);

#if LIBRA_VERSION == 2
        // Reset motion state
        motion_controller_->Reset();
        ReportMotionStatus();
        free_mode_ = false;
        emit FreeModeChanged(false);
#endif

        // Destructing hebi::Group automatically cleans it up
        group_.reset();
        command_->clear();

        logger_->Info("Disconnected from actuator(s)");
        emit Connected(false);
    }
}

/**
 * @brief Sets movement targets/trajectories for all connected actuators and
 *        begins the motion.
 *
 * @param target Target angles (deg, absolute) for all actuators
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

#if LIBRA_VERSION == 2
    if (free_mode_) {
        // A "move" may still command the EPOS yaw joint, so don't error
        logger_->Warn("Ignoring target while in free mode");
        return;
    }
#endif

    // Prepare targets and begin motion
    const Eigen::VectorXd target_pos =
        Eigen::Map<const Eigen::VectorXd>(target.data(), target.size())
        * kDegToRad;

#if LIBRA_VERSION == 1
    group_->getNextFeedback(*feedback_);  // use latest position to avoid jump
    StartTrajectory(feedback_->getPosition(), target_pos);
#elif LIBRA_VERSION == 2
    // Motion controller does the work and requests the movement. It works in
    // joint space (since it relies on specific kinematics)
    std::vector<double> target_joint(n_actuators_);
    for (int i = 0; i < n_actuators_; ++i) {
        target_joint[i] = Joint::ActuatorToJointSimple(JointFromActuator(
                                                           enum_names_[i]),
                                                       target_pos(i));
    }
    motion_controller_->SetTarget(target_joint);
#endif
}

/**
 * @brief Halts the trajectories of all actuators.
 */
void HebiThread::Stop() {
    if (group_ == nullptr) {
        return;  // do nothing
    }

    // End trajectory following; the arm holds its last commanded pose
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_.reset();
        t_trajectory_s_ = 0.0;
    }

#if LIBRA_VERSION == 2
    motion_controller_->Stop();
#endif

    logger_->Info("Trajectory stopped");
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
        emit ErrorThrown("HEBI - Can't stop logging; not connected!");
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
 * @brief Sets the state of the central joint's torque control.
 *
 * @param enabled Whether to control torque experienced by the central joint
 */
void HebiThread::EnableTorqueControl(const bool& enabled) {
#if LIBRA_VERSION == 2
    motion_controller_->EnableTorqueComp(enabled);  // logs by itself
#else
    torque_control_en_ = enabled;

    if (!torque_control_en_) {
        // "Manual" mode: always allow movement while ATC is disabled
        movement_en_ = true;
        playback_rate_ = 1.0;  // full speed
    }

    logger_->Info("Automatic torque compensation "
                  + std::string(enabled ? "enabled" : "disabled"));
#endif
}

/**
 * @brief Sets the bounds for the central joint's torque control.
 *
 * @param lower_bound Lower torque bound (Nm)
 * @param upper_bound Upper torque bound (Nm)
 */
void HebiThread::SetTorqueCompBounds(const double& lower_bound,
                                     const double& upper_bound) {
#if LIBRA_VERSION == 2
    motion_controller_->SetTorqueCompBounds(lower_bound, upper_bound);  // logs
#else
    torque_comp_lower_bound_ = lower_bound;
    torque_comp_upper_bound_ = upper_bound;

    logger_->Info("Torque compensation bounds updated: lower="
                  + FormatDouble(lower_bound)
                  + " Nm, upper=" + FormatDouble(upper_bound) + " Nm");
#endif
}

#if LIBRA_VERSION == 2
/**
 * @brief Selects what moves the pitch joint.
 *
 * @param enabled true = CW moves the arm and actuator takes over near target
 *                (CW-primary); false = actuator and CW work together (tandem)
 *
 * @note Motion controller refuses CW-primary until the arm is calibrated, so
 *       the mode actually in effect is reported back for the UI.
 */
void HebiThread::EnableCwPrimaryMode(const bool& enabled) {
    motion_controller_->SetMotionMode(enabled ? Motion::MotionMode::kCwPrimary
                                              : Motion::MotionMode::kTandem);

    emit MotionModeChanged(motion_controller_->GetMotionMode()
                           == Motion::MotionMode::kCwPrimary);
}

/**
 * @brief Turns free mode on or off. While on, nothing is sent to the actuator,
 *        allowing it to rotate freely.
 *
 * @param enabled Whether to stop commanding the actuator
 */
void HebiThread::EnableFreeMode(const bool& enabled) {
    if (enabled && group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't enter free mode; not connected!");
        emit FreeModeChanged(false);
        return;
    }

    if (enabled && !free_mode_) {
        Stop();
        motion_controller_->Reset();
        free_mode_ = true;
        logger_->Info("Free mode ON: actuator will release in "
                      + std::to_string(group_->getCommandLifetimeMs()) + " ms");
    } else if (!enabled && free_mode_) {
        free_mode_ = false;
        logger_->Info("Free mode OFF: holding current pose");
    }

    emit FreeModeChanged(free_mode_);
}

/**
 * @brief Turns the inertia terms of the torque feedforward on or off.
 *
 * @param enabled Whether to include the inertia terms
 */
void HebiThread::EnableInertiaFeedforward(const bool& enabled) {
    motion_controller_->EnableInertiaFeedforward(enabled);  // logs by itself
}

/**
 * @brief Sets the CW-primary hand-off band.
 *
 * @param deg Band around target angle (deg, joint space)
 */
void HebiThread::SetHandoffBand(const double& deg) {
    // Motion controller logs success
    if (!motion_controller_->SetHandoffBand(deg * kDegToRad)) {
        emit ErrorThrown("HEBI - Invalid hand-off band: " + QString::number(deg)
                         + " deg");
    }
}

//------------------------------------------------------------------------------
// !Calibration (slots)
//------------------------------------------------------------------------------

/**
 * @brief Starts the level calibration (arm must already be level and at rest).
 */
void HebiThread::StartLevelCalibration() {
    if (!CanCalibrate()) {
        return;
    }
    if (!motion_controller_->StartLevelCalibration()) {
        emit ErrorThrown("HEBI - Can't start the level calibration (see the "
                         "log for the reason)!");
        return;
    }
    calibration_running_ = true;
}

/**
 * @brief Starts the two-angle calibration; the motion controller moves the arm
 *        to both angles itself.
 *
 * @param deg_a First angle (deg, joint space, up = +)
 * @param deg_b Second angle (deg, joint space, up = +)
 */
void HebiThread::StartTwoAngleCalibration(const double& deg_a,
                                          const double& deg_b) {
    if (!CanCalibrate()) {
        return;
    }
    if (!motion_controller_->StartTwoAngleCalibration(deg_a * kDegToRad,
                                                      deg_b * kDegToRad)) {
        emit ErrorThrown("HEBI - Can't start the two-angle calibration (see "
                         "the log for the reason)!");
        return;
    }
    calibration_running_ = true;
}

/**
 * @brief Starts the friction calibration; the motion controller sweeps the arm
 *        up and down the window at several speeds itself.
 *
 * @param deg_a One end of the window (deg, joint space, up = +)
 * @param deg_b Other end of the window (deg, joint space, up = +)
 *
 * @note Sweep speeds are fractions of `kMaxVel`, so they stay within the
 *       trajectory speed limit.
 */
void HebiThread::StartFrictionCalibration(const double& deg_a,
                                          const double& deg_b) {
    if (!CanCalibrate()) {
        return;
    }
    if (!motion_controller_->StartFrictionCalibration(deg_a * kDegToRad,
                                                      deg_b * kDegToRad,
                                                      kMaxVel)) {
        emit ErrorThrown("HEBI - Can't start the friction calibration (see "
                         "the log for the reason)!");
        return;
    }
    calibration_running_ = true;
}

/**
 * @brief Cancels a running calibration (no effect otherwise).
 */
void HebiThread::CancelCalibration() {
    if (!calibration_running_) {
        logger_->Info("No calibration running; nothing to cancel");
        return;
    }
    motion_controller_->CancelCalibration();  // logs by itself
}
#endif
