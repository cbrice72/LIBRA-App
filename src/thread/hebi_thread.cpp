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
#include <limits>
#include <optional>
#include <sstream>
#include <thread>

// Other Library Headers
#include "log_file.hpp"  // HEBI
#include "lookup.hpp"    // HEBI
#include <QDir>          // Qt::Core

// Project Headers
#if LIBRA_VERSION == 2
# include "libra2_motion_controller.h"
#endif
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

// Control Loop (see `run()`)

constexpr unsigned long kLoopPeriodMs = 10;             // ms (100 Hz)
constexpr double kLoopPeriod = kLoopPeriodMs / 1000.0;  // s

#if LIBRA_VERSION == 2
// TODO: guess; how often the debug log shows the controller's model values
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
std::string FormatDouble(double value, int precision = 2) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

#if LIBRA_VERSION == 2
/**
 * @brief Formats an optional value, or "n/a" if it is empty.
 *
 * @param value Value to format
 * @param precision Number of decimal places
 */
std::string FormatOptional(const std::optional<double>& value,
                           int precision = 2) {
    return value ? FormatDouble(*value, precision) : "n/a";
}

/**
 * @brief Encodes a CW command for `ReportArmTorque`.
 *
 * @param command What the CW should do
 * @return std::optional<double> 0 = drain, pi = fill, none = idle. (This is the
 *         torque-direction encoding LIBRA-I uses;
 *         `WaterController::UpdateTorqueFeedback` only checks "0 / non-zero /
 *         none".)
 */
std::optional<double> CwCommandToReport(Motion::CwCommand command) {
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
      n_actuators_(names_.size())
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

#if LIBRA_VERSION == 2
    // The controller shares this logger, so `SetDebugMode()` reaches it too
    motion_controller_ = std::make_unique<Motion::Libra2MotionController>(
        *logger_);
    if (motion_controller_->NumJoints()
        != static_cast<std::size_t>(n_actuators_)) {
        logger_->Error("Motion controller joint count != number of actuators; "
                       "its commands will be ignored");
    }
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

#if LIBRA_VERSION == 1
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

# if LIBRA_VERSION == 1
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
# endif

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
        vel_scale_ = 0.5;  // helps avoid sudden stop if upper bound is exceeded

    } else {
        // Once arm torque reaches an acceptable level, disable fluid system
        // balancing and re-enable movement
        emit ReportArmTorque(std::nullopt);
        if (!movement_en_) {
            movement_en_ = true;
            vel_scale_ = 1.0;

            // Reset trajectory clock to pre-stop time
            t_trajectory_start_ = std::chrono::steady_clock::now()
                                  - t_trajectory_elapsed_;

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
bool HebiThread::ExecuteMovement(std::chrono::duration<double> dt,
                                 Eigen::VectorXd& cmd_pos,
                                 Eigen::VectorXd& cmd_vel,
                                 Eigen::VectorXd& cmd_acc,
                                 Eigen::VectorXd& cmd_eff) {
    // Initialize persistent variables
    std::chrono::steady_clock::duration t_trajectory = t_trajectory_elapsed_;

    // Determine movement command
    if (movement_en_ && trajectory_ != nullptr) {
        t_trajectory = std::chrono::steady_clock::now() - t_trajectory_start_;
        // Track trajectory progress in case movement is disabled
        // mid-trajectory (e.g., by ATC).
        t_trajectory_elapsed_ = t_trajectory;

        if (std::chrono::duration<double>(t_trajectory).count()
            < trajectory_->getDuration()) {
            // --- MODE 1: TRAJECTORY FOLLOWING ---

            // Build next step of trajectory
            // TODO: this used to pass `t_trajectory.count()` (clock ticks,
            //       i.e., nanoseconds), but HEBI expects seconds. Verify on
            //       hardware (see the warning in StartTrajectory()).
            trajectory_
                ->getState(std::chrono::duration<double>(t_trajectory).count(),
                           &cmd_pos, &cmd_vel, &cmd_acc);
            command_->setPosition(cmd_pos);
            command_->setVelocity(cmd_vel * vel_scale_);
        } else {
            // --- MODE 2: POSITION HOLDING ---

            // Trajectory is complete
            trajectory_.reset();

            logger_->Debug("Trajectory complete");
        }
    }

    // Send movement command
    return group_->sendCommand(*command_);
}
#endif  // LIBRA_VERSION == 1

/**
 * @brief Starts a rest-to-rest trajectory between two poses.
 *
 * @param start_pos Where the trajectory starts (rad, actuator space)
 * @param target_pos Where it ends (rad, actuator space)
 *
 * @note Called from the GUI thread (`SetTarget()`) and, for LIBRA-II, from the
 *       control loop (moves requested by the motion controller).
 *       TODO: `trajectory_` is shared between those threads without a lock
 *       (pinned thread-safety item).
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
    time << 0, std::max(max_difference / kMaxVel, kLoopPeriod);
#else
    time << 0, max_difference / kMaxVel;
#endif

    // Log start time and create trajectory
#if LIBRA_VERSION == 1
    t_trajectory_start_ = std::chrono::steady_clock::now();
    t_trajectory_elapsed_ = std::chrono::steady_clock::duration::zero();
#elif LIBRA_VERSION == 2
    t_trajectory_s_ = 0.0;
#endif
    trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(time, pos,
                                                                      &vel,
                                                                      &accel);

    logger_->Debug("Set trajectory target(s) to " + trajectory_ss.str()
                   + " rad");
}

#if LIBRA_VERSION == 2
/**
 * @brief LIBRA-II control cycle: hands the measurements to the motion
 *        controller, applies what it returns, and sends the command.
 *
 * @param dt Time since last command loop
 * @param cmd_pos Trajectory position buffer (rad, actuator space)
 * @param cmd_vel Velocity reference buffer (rad/s, actuator space); holds the
 *                PREVIOUS cycle's reference on entry
 * @param cmd_acc Acceleration reference buffer (rad/s^2, actuator space);
 *                same
 * @param cmd_eff Effort command buffer (Nm, actuator space)
 * @return true Command sent
 * @return false Sending the command failed
 *
 * @note The motion controller only sees joint space; converting to and from
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

    // Without a trajectory there is no reference (e.g., after Stop())
    if (trajectory_ == nullptr) {
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
        // TODO: casting Actuator::Name to Joint::Name is bug-prone (as in
        //       GetJointFeedbackMap)
        const auto joint = static_cast<Joint::Name>(enum_names_[i]);

        state.pos[i] = Joint::ActuatorToJointSimple(joint,
                                                    feedback_->getPosition()[i]);
        state.vel[i] = Joint::ActuatorToJointSimple(joint,
                                                    feedback_->getVelocity()[i]);
        state.torque_actuator[i] =
            Joint::ActuatorToJointSimple(joint, feedback_->getEffort()[i]);
        state.ref_vel[i] = Joint::ActuatorToJointSimple(joint, cmd_vel[i]);
        state.ref_acc[i] = Joint::ActuatorToJointSimple(joint, cmd_acc[i]);
    }
    state.trajectory_active = (trajectory_ != nullptr);

    const Motion::Libra2Command cmd = motion_controller_->Update(state);
    if (cmd.mode.size() != n) {
        // The controller already logged the problem; keep the last command
        return group_->sendCommand(*command_);
    }

    // Pass the CW command on to the water controller
    if (cmd.cw_command) {
        emit ReportArmTorque(CwCommandToReport(*cmd.cw_command));
    }

    // A move requested by the controller (calibration, CW-primary hand-off)
    if (cmd.move_to && cmd.move_to->size() == n) {
        Eigen::VectorXd move_pos(n_actuators_);
        for (int i = 0; i < n_actuators_; ++i) {
            move_pos(i) = Actuator::JointToActuatorSimple(enum_names_[i],
                                                          (*cmd.move_to)[i]);
        }
        StartTrajectory(feedback_->getPosition(), move_pos);
    }

    // While the CW moves the arm (effort only), the trajectory waits, parked
    // (clock frozen, nothing sent), until the hand-off replaces it
    // TODO: DECISION (not asked): `SetTarget()` always starts a trajectory,
    //       even in CW-primary mode, so it has to be parked. Every way out of
    //       CW-primary mode either replaces it (`move_to`) or drops it
    //       (`Stop()`), but if a new path ever skips both, the parked
    //       trajectory would resume where its clock stopped.
    bool arm_free = false;
    for (int i = 0; i < n_actuators_; ++i) {
        arm_free = arm_free
                   || (cmd.mode[i] == Motion::CommandMode::kEffortOnly);
    }

    // Trajectory following at the controller's playback rate, or holding
    const double rate = std::clamp(cmd.playback_rate, 0.0, 1.0);
    if (arm_free) {
        cmd_vel.setZero();
        cmd_acc.setZero();
        vel_cmd_.setZero();
    } else if (trajectory_ != nullptr) {
        // --- MODE 1: TRAJECTORY FOLLOWING ---

        // The clock stops while playback is paused and resumes where it was
        t_trajectory_s_ += dt_s * rate;

        if (t_trajectory_s_ < trajectory_->getDuration()) {
            trajectory_->getState(t_trajectory_s_, &cmd_pos, &cmd_vel, &cmd_acc);

            // Playing at `rate` scales velocity by rate and acceleration by
            // rate^2, so the reference stays consistent with the position
            cmd_vel *= rate;
            cmd_acc *= rate * rate;
            vel_cmd_ = cmd_vel;
        } else {
            // Trajectory is complete: hold the exact final pose
            trajectory_->getState(trajectory_->getDuration(), &cmd_pos,
                                  &cmd_vel, &cmd_acc);
            cmd_vel.setZero();
            cmd_acc.setZero();
            vel_cmd_.setZero();
            trajectory_.reset();

            logger_->Debug("Trajectory complete");
        }
        pos_cmd_ = cmd_pos;
    } else {
        // --- MODE 2: POSITION HOLDING ---

        vel_cmd_.setZero();
    }

    // Per-joint command. NaN = no command, which is ASSUMED to leave the
    // joint free (no motor output).
    // TODO: not confirmed for Strategy 4 with NaN position/velocity/effort;
    //       verify on hardware (support the arm the first time).
    for (int i = 0; i < n_actuators_; ++i) {
        if (cmd.mode[i] == Motion::CommandMode::kEffortOnly) {
            pos_cmd_(i) = kNone;
            vel_cmd_(i) = kNone;
        } else if (std::isnan(pos_cmd_(i))) {
            // Just left effort-only mode: hold where the joint is now
            pos_cmd_(i) = feedback_->getPosition()[i];
        }

        // Effort feedforward, joint space -> actuator space (HEBI adds the
        // commanded effort to the position loop's output)
        const auto& torque = cmd.torque_actuator_cmd[i];
        cmd_eff(i) = torque ? Actuator::JointToActuatorSimple(enum_names_[i],
                                                              *torque)
                            : kNone;
    }
    command_->setPosition(pos_cmd_);
    command_->setVelocity(vel_cmd_);
    command_->setEffort(cmd_eff);

    // Periodic model log: what the controller believes vs. what is measured
    diag_log_elapsed_s_ += dt_s;
    if (diag_log_elapsed_s_ >= kDiagnosticsPeriod) {
        diag_log_elapsed_s_ = 0.0;

        const auto& d = cmd.diagnostics;
        logger_->Debug(
            "Model - m_cw=" + FormatOptional(d.cw_mass_kg) + " kg, tau_ext,g="
            + FormatOptional(d.torque_ext_g) + " Nm, m_cw,bal(target)="
            + FormatOptional(d.cw_mass_tgt_kg) + " kg, delta_m="
            + FormatOptional(d.cw_mass_change_kg) + " kg, tau_ext,g(target)="
            + FormatOptional(d.torque_ext_g_tgt) + " Nm");
    }

    ReportMotionStatus();
    return group_->sendCommand(*command_);
}

/**
 * @brief Tells the UI when movement starts/stops and when a calibration ends.
 *
 * @note "Moving" covers a trajectory (including the one parked during
 *       CW-primary motion) and a running calibration; the UI disables mode
 *       switching while it is true.
 */
void HebiThread::ReportMotionStatus() {
    const auto status = motion_controller_->GetCalibrationStatus();

    // Exactly one `CalibrationFinished` per accepted start (cancel = failure)
    if (calibration_running_ && !Motion::IsCalibrationRunning(status)) {
        calibration_running_ = false;
        emit CalibrationFinished(status
                                 == Motion::CalibrationStatus::kSucceeded);
    }

    const bool moving = (trajectory_ != nullptr) || calibration_running_;
    if (moving != moving_) {
        moving_ = moving;
        emit MovementStateChanged(moving);
    }
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
 * @see CheckTorqueControl(), ExecuteMovement() (LIBRA-I),
 *      ExecuteMotionController() (LIBRA-II), SendFeedback()
 */
void HebiThread::run() {
    logger_->Debug("Starting thread");

    // Initialize buffers for loop efficiency
    Eigen::VectorXd cmd_pos = Eigen::VectorXd::Zero(n_actuators_);
    Eigen::VectorXd cmd_vel = Eigen::VectorXd::Zero(n_actuators_);
    Eigen::VectorXd cmd_acc = Eigen::VectorXd::Zero(n_actuators_);  // <- traj.
    Eigen::VectorXd cmd_eff = Eigen::VectorXd::Zero(n_actuators_);  // -> cmd.

    // For dynamic compensation (DynamicComp)
    auto last_loop_time = std::chrono::steady_clock::now();

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        if (group_ == nullptr) {
            emit ReportStatus(GetStatus(), type_);  // "Not Connected"
            // Keep the first `dt` after `Connect()` small
            last_loop_time = std::chrono::steady_clock::now();
            QThread::msleep(kLoopPeriodMs);
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
        QThread::msleep(kLoopPeriodMs);
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
    // Same pose for the loop's position hold; clean controller and clock
    pos_cmd_ = feedback_->getPosition();
    vel_cmd_ = Eigen::VectorXd::Zero(n_actuators_);
    t_trajectory_s_ = 0.0;
    motion_controller_->Reset();
#endif

    logger_->Debug("Connection successful");
    emit Connected(true);
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

#if LIBRA_VERSION == 2
        // Drop controller state (this cancels a calibration) and stop the CW
        // TODO: DECISION (not asked): the CW is told to idle here so water
        //       does not keep flowing after a disconnect. LIBRA-I does not do
        //       this.
        motion_controller_->Reset();
        emit ReportArmTorque(std::nullopt);
        ReportMotionStatus();
#endif

        // Destructing hebi::Group automatically cleans it up
        group_.reset();
        command_->clear();

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
    t_trajectory_start_ = std::chrono::steady_clock::now();
    trajectory_ =
        hebi::trajectory::Trajectory::createUnconstrainedQp(t_waypoint, pos,
                                                            nullptr, nullptr);
    */

    // Start from the freshest pose (targets are assumed to be in "actuator
    // space")
    group_->getNextFeedback(*feedback_);
    const Eigen::VectorXd start_pos = feedback_->getPosition();
    const Eigen::VectorXd target_pos =
        Eigen::Map<const Eigen::VectorXd>(target.data(), target.size())
        * kDegToRad;

#if LIBRA_VERSION == 2
    // The motion controller works in joint space, so give it the same target
    // converted (this also cancels a running calibration)
    Motion::JointValues target_joint(n_actuators_);
    for (int i = 0; i < n_actuators_; ++i) {
        // TODO: casting Actuator::Name to Joint::Name is bug-prone (as in
        //       GetJointFeedbackMap)
        target_joint[i] = Joint::ActuatorToJointSimple(static_cast<Joint::Name>(
                                                           enum_names_[i]),
                                                       target_pos(i));
    }
    motion_controller_->SetTarget(target_joint);
#endif

    StartTrajectory(start_pos, target_pos);
}

/**
 * @brief Halts the trajectories of all actuators.
 */
void HebiThread::Stop() {
    if (group_ == nullptr) {
        return;  // do nothing
    }

    trajectory_.reset();
#if LIBRA_VERSION == 1
    t_trajectory_elapsed_ = std::chrono::steady_clock::duration::zero();
#elif LIBRA_VERSION == 2
    t_trajectory_s_ = 0.0;
    motion_controller_->Stop();  // drops the target and cancels a calibration
#endif

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
        // "Manual" mode: only force allow movement if torque compensation is disabled
        movement_en_ = true;
    }

    logger_->Debug("ATC - Automatic torque compensation "
                   + std::string(enabled ? "enabled" : "disabled"));
#endif
}

/**
 * @brief Sets the bounds for the central joint's torque control.
 *
 * @param lower_bound Upper torque bound, in Nm
 * @param upper_bound Lower torque bound, in Nm
 */
void HebiThread::SetTorqueCompBounds(const double& lower_bound,
                                     const double& upper_bound) {
#if LIBRA_VERSION == 2
    motion_controller_->SetTorqueCompBounds(lower_bound, upper_bound);  // logs
#else
    torque_comp_lower_bound_ = lower_bound;
    torque_comp_upper_bound_ = upper_bound;

    logger_->Debug("ATC - Torque compensation bounds updated: lower="
                   + FormatDouble(lower_bound)
                   + " Nm, upper=" + FormatDouble(upper_bound) + " Nm");
#endif
}

#if LIBRA_VERSION == 2
/**
 * @brief Selects who moves the pitch joint.
 *
 * @param enabled true = the CW moves the arm and the actuator takes over near
 *                the target (CW-primary); false = actuator and CW work
 *                together (tandem)
 *
 * @note The controller refuses CW-primary until the arm is calibrated, so the
 *       mode actually in effect is reported back for the UI to sync with.
 */
void HebiThread::EnableCwPrimaryMode(const bool& enabled) {
    motion_controller_->SetMotionMode(enabled ? Motion::MotionMode::kCwPrimary
                                              : Motion::MotionMode::kTandem);

    emit MotionModeChanged(motion_controller_->GetMotionMode()
                           == Motion::MotionMode::kCwPrimary);
}

/**
 * @brief Sets the CW-primary hand-off band.
 *
 * @param deg Angle band around the target, in degrees (joint space)
 */
void HebiThread::SetHandoffBand(const double& deg) {
    motion_controller_->SetHandoffBand(deg * kDegToRad);
}

/**
 * @brief Gives the motion controller the latest CW volume.
 *
 * @param liters CW volume, in liters (from the TankWidget)
 */
void HebiThread::UpdateCounterweightVolume(const double& liters) {
    motion_controller_->SetCwVolume(liters);
}

//------------------------------------------------------------------------------
// !Calibration (slots)
//------------------------------------------------------------------------------

/**
 * @brief Starts the level calibration. The arm must already be level and at
 *        rest.
 *
 * @note `CalibrationFinished` is emitted exactly once per call (also when the
 *       calibration is refused or cancelled).
 */
void HebiThread::StartLevelCalibration() {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't calibrate; not connected!");
        emit CalibrationFinished(false);
        return;
    }

    if (!motion_controller_->StartLevelCalibration()) {
        emit CalibrationFinished(false);  // the controller logged the reason
        return;
    }
    calibration_running_ = true;
}

/**
 * @brief Starts the two-angle calibration. The controller moves the arm to
 *        both angles by itself.
 *
 * @param deg_a First angle, in degrees (joint space, positive = up)
 * @param deg_b Second angle, in degrees (joint space, positive = up)
 *
 * @note `CalibrationFinished` is emitted exactly once per call.
 */
void HebiThread::StartTwoAngleCalibration(const double& deg_a,
                                          const double& deg_b) {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't calibrate; not connected!");
        emit CalibrationFinished(false);
        return;
    }

    if (!motion_controller_->StartTwoAngleCalibration(deg_a * kDegToRad,
                                                      deg_b * kDegToRad)) {
        emit CalibrationFinished(false);  // the controller logged the reason
        return;
    }
    calibration_running_ = true;
}

/**
 * @brief Cancels a running calibration (no effect otherwise).
 */
void HebiThread::CancelCalibration() {
    motion_controller_->CancelCalibration();
}
#endif
