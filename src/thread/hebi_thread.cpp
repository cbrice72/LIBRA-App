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
#include "group_info.hpp"  // HEBI
#include "log_file.hpp"    // HEBI
#include "lookup.hpp"      // HEBI
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
 * !Getters & Setters
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
constexpr double kNone = std::numeric_limits<double>::quiet_NaN();  // no command

// Safety

constexpr bool kAllowFreeJoints = (LIBRA_VERSION == 2);

#if LIBRA_VERSION == 2
constexpr double kMaxSweepVel = 0.1047;  // for friction sweep, rad/s (~6 deg/s)
#endif

// Motion Controller Logging

#if LIBRA_VERSION == 2
constexpr double kDiagnosticsPeriod = 3.0;  // s
#endif

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

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
 * @brief Gets the lowest velocity limit (rad/s) in a set of safety
 *        parameters, if any actuator has one.
 *
 * @param safety_cmd Command holding safety parameters (from
 *                   `readSafetyParameters()`)
 */
std::optional<double> MaxVelFromSafety(const hebi::GroupCommand& safety_cmd) {
    std::optional<double> max_vel;
    for (std::size_t i = 0; i < safety_cmd.size(); ++i) {
        const auto& actuator = safety_cmd[i].settings().actuator();
        for (const auto* limit :
             {&actuator.velocityLimitMin(), &actuator.velocityLimitMax()}) {
            if (limit->has() && std::isfinite(limit->get())) {
                const double v = std::abs(limit->get());
                max_vel = max_vel ? std::min(*max_vel, v) : v;
            }
        }
    }
    return max_vel;
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
#ifdef BUILD_WITH_ROS2
      rclcpp::Node("hebi_node"),
#endif
#if LIBRA_VERSION == 2
      motion_controller_(std::move(motion_controller)),
#endif
      families_(std::move(families)),
      names_(std::move(names)),
      n_actuators_(names_.size()),
      loop_period_ms_(10)  // 100 Hz
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

    max_vel_ = kMaxVel;  // until safety parameters are loaded

    latest_cmd_pos_ = Eigen::VectorXd::Constant(n_actuators_, kNone);
    latest_cmd_vel_ = Eigen::VectorXd::Zero(n_actuators_);

    // "Translate" string names into corresponding enums for feedback_map
    enum_names_.resize(n_actuators_);
    for (std::size_t i = 0; i < n_actuators_; ++i) {
        enum_names_[i] = Actuator::StringToNameEnum(names_[i]);

        // Every actuator must map to a known joint
        if (!Actuator::IsDifferential(enum_names_[i])
            && JointFromActuator(enum_names_[i]) == Joint::Name::kUndefined) {
            logger_->Error("Actuator " + names_[i]
                           + " has no matching joint; its feedback and "
                             "commands will be ignored");
        }
    }

    // Resize vectors to match number of actuators
    a_vel_.resize(n_actuators_);
    defl_.resize(n_actuators_);
    defl_vel_.resize(n_actuators_);
    volt_.resize(n_actuators_);
    curr_.resize(n_actuators_);
    temp_.resize(n_actuators_);

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

    // One entry per joint name
    for (auto* msg : {&state_msg_, &target_msg_}) {
        msg->name = hebi_names;
        msg->position.resize(hebi_names.size());
        msg->velocity.resize(hebi_names.size());
        msg->effort.resize(hebi_names.size());
    }
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
        if (type == Actuator::Feedback::kCommandPos) {
            val = feedback->getPositionCommand()[i] * kRadToDeg;
        } else if (type == Actuator::Feedback::kCommandVel) {
            val = feedback->getVelocityCommand()[i] * kRadToDeg;
        } else if (type == Actuator::Feedback::kCommandTorque) {
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

/**
 * @brief Runs one control cycle: starts a new target, follows the trajectory
 *        with the selected assistance mode, and sends the resulting command.
 *        False if the command couldn't be sent.
 *
 * @param dt Time since last control cycle
 * @param cmd_pos Commanded position (rad)
 * @param cmd_vel Commanded velocity (rad/s); previous cycle's reference
 * @param cmd_acc Commanded acceleration (rad/s²); previous cycle's reference
 * @param cmd_trq Commanded torque (Nm)
 *
 * @note Only used in `run()`; separate function for readability.
 */
bool HebiThread::ExecuteMovement(std::chrono::duration<double> dt,
                                 Eigen::VectorXd& cmd_pos,
                                 Eigen::VectorXd& cmd_vel,
                                 Eigen::VectorXd& cmd_acc,
                                 Eigen::VectorXd& cmd_trq) {
    const double dt_s = dt.count();

#if LIBRA_VERSION == 2
    // Pitch control released (send nothing)
    if (pitch_released_) {
        latest_cmd_pos_.setConstant(kNone);
        latest_cmd_vel_.setZero();
        emit ReportArmTorque(std::nullopt);
        ReportMovement(false);
        return true;
    }
#endif

    // Determine assistance type + value (e.g., motion controller, simple feedback)
    std::optional<Motion::MotionCommand> assist;
#if LIBRA_VERSION == 2
    if (Motion::UsesMotionController(motion_mode_) || calibration_running_) {
        assist = RunMotionController(dt_s, cmd_vel, cmd_acc);
        if (!assist) {
            return group_->sendCommand(*command_);  // keep last command
        }
    }
#endif
    if (!assist) {
        assist.emplace(Motion::kNumJoints);
        assist->trajectory_rate = UpdateTrajectoryRate();
    }

    // Start the latest target from `SetTarget()`, if any
    // (NOTE: motion controller may request a pause on the current trajectory)
    std::optional<Eigen::VectorXd> target_pos;
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        target_pos.swap(pending_target_pos_);
    }
    if (target_pos) {
        StartTrajectory(HeldPosition(), *target_pos);
    }

    // Determine motion command and any required assistance
    const bool following = FollowTrajectory(dt_s, cmd_pos, cmd_vel, cmd_acc,
                                            assist->trajectory_rate);

    for (int i = 0; i < n_actuators_; ++i) {
        const auto joint = JointFromActuator(enum_names_[i]);
        const bool is_simple_joint = (joint != Joint::Name::kUndefined);

        // [LIBRA-II] CW Primary: switch between passive and active control
        // NOTE: Gated by `kAllowFreeJoints`, since this is unsafe on LIBRA-I
        if (kAllowFreeJoints && is_simple_joint
            && assist->mode[joint] == Motion::CommandMode::kFree) {
            latest_cmd_pos_(i) = kNone;
            latest_cmd_vel_(i) = kNone;
        } else if (std::isnan(latest_cmd_pos_(i))) {
            latest_cmd_pos_(i) = feedback_->getPosition()[i];
        }

        // NOTE: Currently not used by LIBRA-I, but left in for future use
        cmd_trq(i) = kNone;
        if (is_simple_joint && assist->ff_torque[joint]) {
            const double ff_torque = *assist->ff_torque[joint];
            cmd_trq(i) = Actuator::JointToActuatorSimple(enum_names_[i],
                                                         ff_torque);
        }
    }

    // Command the trajectory (or hold)
    command_->setPosition(latest_cmd_pos_);
    command_->setVelocity(latest_cmd_vel_);
    command_->setEffort(cmd_trq);

    // NOTE: "Pause" still counts as moving, since the trajectory isn't done
    ReportMovement(move_requested_.exchange(false) || following
                   || assist->moving);

    return group_->sendCommand(*command_);
}

/**
 * @brief Updates trajectory rate multiplier (0.0 = paused, 1.0 = full speed)
 *        based on torque feedback on the central joint (LIBRA-I's 2-DoF joint,
 *        or LIBRA-II's pitch joint), commanding the CW toward balance.
 *
 * @note Manual mode always gets full speed.
 *
 * @note LIBRA-II only uses this outside motion controller modes, since the
 *       motion controller has its own reactive counterweight.
 *
 * @note Only used in `ExecuteMovement()`; separate function for readability.
 */
double HebiThread::UpdateTrajectoryRate() {
    if (motion_mode_ == Motion::MotionMode::kManual) {
        return 1.0;
    }

    // Retrieve torque magnitude and direction (i.e., polar coords)
    const auto& eff = feedback_->getEffort();

#if LIBRA_VERSION == 1
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
#elif LIBRA_VERSION == 2
    const double arm_pitch_torque =
        Joint::ActuatorToJointSimple(Joint::Name::kPitch,
                                     eff[Actuator::Name::kPitch]);
    const double arm_torque_r = std::abs(arm_pitch_torque);

    // NOTE: Same fill/drain logic as the motion controller's reactive CW
    const double arm_torque_theta = *CwCommandToReport(
        (arm_pitch_torque > 0.0) ? Motion::CwCommand::kFill
                                 : Motion::CwCommand::kDrain);
#endif

    // Apply 3-tier hysteresis logic
    // TODO: Ideally we should be predicting how a trajectory will affect the
    //       central joint's feedback torque, maybe with ROS2 or something.
    if (arm_torque_r > reactive_cw_upper_bound_) {
        // If arm torque is too high, disable movement before reporting the
        // torque direction to WaterController for CW balancing
        if (movement_en_) {
            movement_en_ = false;

            logger_->Info("Movement disabled due to high torque ("
                          + FormatDouble(arm_torque_r) + " Nm > "
                          + FormatDouble(reactive_cw_upper_bound_) + " Nm)");
        } else {  // movement is disabled
            emit ReportArmTorque(arm_torque_theta);
        }

    } else if (arm_torque_r >= reactive_cw_lower_bound_) {
        // While significant arm torque remains, keep reporting its direction
        // to WaterController
        emit ReportArmTorque(arm_torque_theta);
        trajectory_rate_ =
            kReactiveCwSlowRate;  // avoids sudden stop if upper bound is hit

    } else {
        // Once arm torque reaches an acceptable level, disable fluid system
        // balancing and re-enable movement
        emit ReportArmTorque(std::nullopt);
        trajectory_rate_ = 1.0;  // full speed
        if (!movement_en_) {
            movement_en_ = true;

            logger_->Info("Movement re-enabled (" + FormatDouble(arm_torque_r)
                          + " Nm < " + FormatDouble(reactive_cw_lower_bound_)
                          + " Nm)");
        }
    }

    return movement_en_ ? trajectory_rate_ : 0.0;
}

/**
 * @brief Starts a trajectory between two poses.
 *
 * @param start_pos Start pose (rad)
 * @param target_pos End pose (rad)
 *
 */
void HebiThread::StartTrajectory(const Eigen::VectorXd& start_pos,
                                 const Eigen::VectorXd& target_pos) {
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
    // (NOTE: HEBI trajectories are minimum-jerk (see link below), which peak
    //        mid-move at 15/8 their average speed; the planned move is
    //        stretched by 15/8 so the peak happens at `max_vel_`)
    // https://files.hebi.us/docs/matlab/hebi-matlab-2.4/HebiTrajectoryGenerator.setAlgorithm.html
    constexpr double kPeakToAvgVel = 15.0 / 8.0;
    const double t_move_s = kPeakToAvgVel * max_difference / max_vel_;

    // A zero-length move would give a zero-duration (invalid) trajectory, so
    // allow at least one loop period (the shortest the loop can follow)
    Eigen::VectorXd time(2);
    time << 0, std::max(t_move_s, loop_period_ms_ / 1000.0);

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

/**
 * @brief Gets the pose the position loop is holding
 *
 * @note Starting from measured pose instead might cause the actuators to "jump"
 *       the position command by the position error, suddenly dropping any
 *       torque from the P gain.
 */
Eigen::VectorXd HebiThread::HeldPosition() const {
    Eigen::VectorXd held = feedback_->getPosition();

    if (latest_cmd_pos_.size() != held.size()) {
        return held;  // no previous command exists
    }

    for (int i = 0; i < held.size(); ++i) {
        if (std::isfinite(latest_cmd_pos_(i))) {
            held(i) = latest_cmd_pos_(i);
        }
    }
    return held;
}

/**
 * @brief Follows the trajectory for one cycle at the given rate, updating the
 *        position/velocity command to match, or to hold the final pose once it
 *        ends. False once no trajectory remains.
 *
 * @param dt_s Time since last control cycle (s)
 * @param cmd_pos Commanded position (rad)
 * @param cmd_vel Commanded velocity (rad/s); previous cycle's reference
 * @param cmd_acc Commanded acceleration (rad/s²); previous cycle's reference
 * @param trajectory_rate Trajectory speed, [0.0, 1.0] (1.0 = normal, 0.0 = paused)
 *
 * @note A reduced rate scales velocity and acceleration, keeping the reference
 *       consistent with position.
 *
 * @note Only used in `ExecuteMovement()`; separate function for readability.
 */
bool HebiThread::FollowTrajectory(const double& dt_s, Eigen::VectorXd& cmd_pos,
                                  Eigen::VectorXd& cmd_vel,
                                  Eigen::VectorXd& cmd_acc,
                                  const double& trajectory_rate) {
    const double rate = std::clamp(trajectory_rate, 0.0, 1.0);

    const std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (trajectory_ == nullptr) {
        latest_cmd_vel_.setZero();  // hold last commanded pose
        return false;
    }

    // Check trajectory state
    t_trajectory_s_ += dt_s * rate;
    if (t_trajectory_s_ < trajectory_->getDuration()) {
        // Still running; update commands
        trajectory_->getState(t_trajectory_s_, &cmd_pos, &cmd_vel, &cmd_acc);
        cmd_vel *= rate;
        cmd_acc *= rate * rate;

        latest_cmd_pos_ = cmd_pos;
        latest_cmd_vel_ = cmd_vel;
        return true;
    }

    // Trajectory complete; hold final pose
    trajectory_->getState(trajectory_->getDuration(), &cmd_pos, &cmd_vel,
                          &cmd_acc);
    cmd_vel.setZero();
    cmd_acc.setZero();
    latest_cmd_pos_ = cmd_pos;
    latest_cmd_vel_.setZero();
    trajectory_.reset();

    logger_->Info("Trajectory complete");
    return false;
}

#if LIBRA_VERSION == 2
/**
 * @brief Runs the motion controller for one cycle and carries out its
 *        requests.
 *
 * @param dt_s Time since last control cycle (s)
 * @param cmd_vel Commanded velocity (rad/s); previous cycle's reference
 * @param cmd_acc Commanded acceleration (rad/s²); previous cycle's reference
 *
 * @note Motion controller only understands "joint space"; conversion to and
 *       from "actuator space" happens here.
 *
 * @note Only used in `ExecuteMovement()`; separate function for readability.
 */
std::optional<Motion::MotionCommand> HebiThread::RunMotionController(
    const double& dt_s, Eigen::VectorXd& cmd_vel, Eigen::VectorXd& cmd_acc) {
    // Without a trajectory, there's nothing to reference
    bool trajectory_active = false;
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_active = (trajectory_ != nullptr);
    }
    if (!trajectory_active) {
        cmd_vel.setZero();
        cmd_acc.setZero();
    }

    // Get actual and previously commanded values, in joint space
    // (NOTE: NaN = non-HEBI joint)
    Motion::ActualState actual;
    actual.pos.assign(Motion::kNumJoints, kNone);
    actual.vel.assign(Motion::kNumJoints, kNone);
    actual.torque.assign(Motion::kNumJoints, kNone);

    Motion::CommandedState commanded;
    commanded.vel.assign(Motion::kNumJoints, kNone);
    commanded.acc.assign(Motion::kNumJoints, kNone);

    for (int i = 0; i < n_actuators_; ++i) {
        const auto joint = JointFromActuator(enum_names_[i]);
        if (joint == Joint::Name::kUndefined) {
            continue;  // logged on construction
        }

        actual.pos[joint] =
            Joint::ActuatorToJointSimple(joint, feedback_->getPosition()[i]);
        actual.vel[joint] =
            Joint::ActuatorToJointSimple(joint, feedback_->getVelocity()[i]);
        actual.torque[joint] =
            Joint::ActuatorToJointSimple(joint, feedback_->getEffort()[i]);

        commanded.vel[joint] = Joint::ActuatorToJointSimple(joint, cmd_vel[i]);
        commanded.acc[joint] = Joint::ActuatorToJointSimple(joint, cmd_acc[i]);
    }

    // Run the motion controller and get updated command
    Motion::Libra2Command cmd = motion_controller_->Update(actual, commanded,
                                                           dt_s,
                                                           trajectory_active,
                                                           motion_mode_);
    if (cmd.mode.size() != Motion::kNumJoints
        || cmd.ff_torque.size() != Motion::kNumJoints) {
        return std::nullopt;
    }
    if (!Motion::UsesMotionController(motion_mode_)) {
        cmd.ff_torque.assign(Motion::kNumJoints, std::nullopt);
    }

    // Pass CW command to WaterController
    if (cmd.cw_command) {
        emit ReportArmTorque(CwCommandToReport(*cmd.cw_command));
    }

    // Deal with stop request, if any
    if (cmd.stop_trajectory) {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_.reset();
        t_trajectory_s_ = 0.0;
        logger_->Info("Trajectory stopped by motion controller");
    }

    // Deal with move request
    if (cmd.move_to && cmd.move_to->size() == Motion::kNumJoints) {
        const Eigen::VectorXd start_pos = HeldPosition();
        Eigen::VectorXd end_pos = start_pos;
        for (int i = 0; i < n_actuators_; ++i) {
            const auto joint = JointFromActuator(enum_names_[i]);
            if (joint != Joint::Name::kUndefined) {
                end_pos(
                    i) = Actuator::JointToActuatorSimple(enum_names_[i],
                                                         (*cmd.move_to)[joint]);
            }
        }
        StartTrajectory(start_pos, end_pos);
    }

    // Deal with sweep request (replaces above motion plan)
    // NOTE: friction calibration only
    if (cmd.sweep) {
        StartSweep(*cmd.sweep);
    }

    // Log motion controller's model vs. measurements
    t_diag_log_elapsed_s_ += dt_s;
    if (diag_log_en_ && t_diag_log_elapsed_s_ >= kDiagnosticsPeriod) {
        t_diag_log_elapsed_s_ = 0.0;

        const auto& d = cmd.diagnostics;
        logger_->Debug(
            "Model values:\n  m_cw:              " + FormatOptional(d.cw_mass)
            + " kg\n  m_cw,bal(target):  " + FormatOptional(d.cw_mass_tgt)
            + " kg\n  delta_m:           " + FormatOptional(d.cw_mass_change)
            + " kg\n  tau_ext,g:         " + FormatOptional(d.torque_ext_g)
            + " Nm\n  tau_ext,g(target): " + FormatOptional(d.torque_ext_g_tgt)
            + " Nm");
    }

    ReportCalibrationStatus();

    return Motion::MotionCommand(cmd);
}

/**
 * @brief Starts a pitch sweep: rest, ramp up to constant speed, hold it, ramp
 *        down to rest (other joints hold).
 *
 * @param sweep Sweep to follow (joint space)
 *
 * @note With position, speed, and zero acceleration fixed at each waypoint, the
 *       middle segment is exactly constant speed, and each ramp (speed v over
 *       distance d) takes 2·d / v.
 *
 * @note A rejected sweep cancels the calibration that requested it.
 */
void HebiThread::StartSweep(const Motion::SweepRequest& sweep) {
    const double dir = (sweep.to > sweep.from) ? 1.0 : -1.0;
    const double ramp_end = sweep.from + (dir * sweep.ramp_dist);
    const double ramp_start = sweep.to - (dir * sweep.ramp_dist);
    const Eigen::VectorXd start = HeldPosition();

    // Pitch must start before the first ramp ends, at a speed within limits
    double pitch_start = sweep.from;
    for (int i = 0; i < n_actuators_; ++i) {
        if (JointFromActuator(enum_names_[i]) == Joint::Name::kPitch) {
            pitch_start = Joint::ActuatorToJointSimple(Joint::Name::kPitch,
                                                       start[i]);
        }
    }
    if (!(sweep.vel > 0.0) || sweep.vel > max_vel_ || !(sweep.ramp_dist > 0.0)
        || dir * (ramp_start - ramp_end) < 0.0
        || dir * (ramp_end - pitch_start) <= 0.0) {
        logger_->Error("Rejected sweep: speed=" + FormatDouble(sweep.vel, 4)
                       + " rad/s (limit " + FormatDouble(max_vel_, 4)
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
    const std::array<double, 4> pitch_vel = {0.0, dir * sweep.vel,
                                             dir * sweep.vel, 0.0};
    const double t_ramp_s = 2.0 * sweep.ramp_dist / sweep.vel;
    const double t_const_s = std::abs(ramp_start - ramp_end) / sweep.vel;

    Eigen::VectorXd time(4);
    time << 0, t_ramp_s, t_ramp_s + t_const_s, (2.0 * t_ramp_s) + t_const_s;
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
                   + FormatDouble(sweep.vel, 4) + " rad/s");
}

/**
 * @brief Reports relevant calibration updates: in-progress, completion, or error.
 */
void HebiThread::ReportCalibrationStatus() {
    const auto status = motion_controller_->GetCalibrationStatus();

    // Report calibration completion or error
    if (calibration_running_ && !Motion::IsCalibrationRunning(status)) {
        calibration_running_ = false;
        last_progress_.clear();

        const QString message = QString::fromStdString(
            motion_controller_->GetCalibrationMessage());
        emit CalibrationProgressChanged(message, false);

        if (status == Motion::CalibrationStatus::kFailed) {
            emit ErrorThrown("HEBI - " + message);
        }
    }

    // Report calibration progress (e.g., "Step 2 of 5")
    if (calibration_running_) {
        const std::string progress = motion_controller_->GetCalibrationProgress();
        if (!progress.empty() && progress != last_progress_) {
            last_progress_ = progress;
            emit CalibrationProgressChanged(QString::fromStdString(progress),
                                            true);
        }
    }

    // Report calibration level changes (e.g., "uncalibrated" to "calibrated")
    const auto level = motion_controller_->GetCalibration();
    if (calibration_level_.exchange(level) != level) {
        emit CalibrationChanged(level);
    }
}

/**
 * @brief Whether a calibration may start.
 */
bool HebiThread::CanCalibrate() {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't calibrate; not connected!");
        return false;
    }
    if (pitch_released_) {
        emit ErrorThrown("HEBI - Can't calibrate while pitch is released!");
        return false;
    }

    return true;
}
#endif  // LIBRA_VERSION == 2

/**
 * @brief Sets the maximum trajectory speed from the given safety parameters.
 *
 * @param safety_cmd HEBI safety parameters (from `readSafetyParameters()`)
 * @param source Where the parameters came from (for logging purposes)
 *
 * @note Falls back to `kMaxVel` if no actuator has a velocity limit.
 */
void HebiThread::SetMaxVel(const hebi::GroupCommand& safety_cmd,
                           const std::string& source) {
    const auto limit = MaxVelFromSafety(safety_cmd);
    max_vel_ = limit.value_or(kMaxVel);

    if (limit) {
        logger_->Info("Max. trajectory speed: " + FormatDouble(max_vel_, 3)
                      + " rad/s (velocity limit in " + source + ")");
    } else {
        logger_->Warn("No velocity limit in " + source
                      + "; max. trajectory speed: " + FormatDouble(max_vel_, 3)
                      + " rad/s (default)");
    }
}

/**
 * @brief Sends actuator feedback to whoever's listening (Qt/ROS2).
 *
 * @note Command buffers are reused to avoid repeated allocations at 100 Hz.
 */
void HebiThread::SendFeedback() {
    // Report important statuses individually
    emit ReportFeedback(GetJointFeedbackMap(feedback_,
                                            Actuator::Feedback::kCommandPos),
                        Actuator::Feedback::kCommandPos);
    emit ReportFeedback(GetJointFeedbackMap(feedback_,
                                            Actuator::Feedback::kActualPos),
                        Actuator::Feedback::kActualPos);
    emit ReportFeedback(GetJointFeedbackMap(feedback_,
                                            Actuator::Feedback::kActualTorque),
                        Actuator::Feedback::kActualTorque);

    // Report miscellaneous statuses in one batch
    emit ReportTelemetry(GetTelemetry(), type_);

#ifdef BUILD_WITH_ROS2
    // Make actuator state info available to other ROS2 nodes
    PublishState();
#endif
}

/**
 * @brief Telemetry for all connected actuators, as display-ready text
 *        messages (target/actual position and actual torque are sent
 *        separately as signals).
 *
 * @note See the HEBI C++ API page for the full list of available feedback:
 *       https://files.hebi.us/docs/cpp/cpp-3.11.1/classhebi_1_1GroupFeedback.html
 */
QString HebiThread::GetTelemetry() const {
    if (group_ == nullptr) {
        return {"Not Connected"};
    }

    const auto& vel = feedback_->getVelocity();
    const auto& defl = feedback_->getDeflection();
    const auto& defl_vel = feedback_->getDeflectionVelocity();
    const auto& volt = feedback_->getVoltage();
    const auto& curr = feedback_->getMotorCurrent();
    const auto& temp = feedback_->getMotorWindingTemperature();
    // (alternatively, `getBoardTemperature()` for electronics)

    // Convert data, if necessary
    for (int i = 0; i < n_actuators_; ++i) {
        a_vel_[i] = vel[i] * kRadToDeg;
        defl_[i] = defl[i] * kRadToDeg;
        defl_vel_[i] = defl_vel[i] * kRadToDeg;
        volt_[i] = volt[i];
        curr_[i] = curr[i];
        temp_[i] = temp[i];
    }

    // Format telemetry
    std::ostringstream ss;
    std::vector<std::string> header;
    for (const auto& name : names_) {
        header.push_back("[" + name + "]");
    }
    AddTelemetryTextRow(ss, "Actuator Name", header);

    AddTelemetryRow(ss, "Actual Velocity", a_vel_, "deg/s");
    AddTelemetryRow(ss, "Deflection", defl_, "deg");
    AddTelemetryRow(ss, "Deflection Velocity", defl_vel_, "deg/s");
    AddTelemetryRow(ss, "Voltage", volt_, "V");
    AddTelemetryRow(ss, "Current", curr_, "A");
    AddTelemetryRow(ss, "Temperature", temp_, "C");

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

    const auto command_pos_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kCommandPos);
    const auto command_vel_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kCommandVel);
    const auto command_eff_map =
        GetJointFeedbackMap(feedback_, Actuator::Feedback::kCommandTorque);

    // Populate messages and publish
    for (int i = 0; i < n_actuators_; ++i) {
        const auto joint = static_cast<Joint::Name>(i);

        const double actual_pos = actual_pos_map.at(joint);
        const double actual_vel = actual_vel_map.at(joint);
        const double actual_eff = actual_eff_map.at(joint);

        state_msg_.position[i] = actual_pos * kDegToRad;
        state_msg_.velocity[i] = actual_vel * kDegToRad;
        state_msg_.effort[i] = actual_eff;

        const double command_pos = command_pos_map.at(joint);
        const double command_vel = command_vel_map.at(joint);
        const double command_eff = command_eff_map.at(joint);

        target_msg_.position[i] = command_pos * kDegToRad;
        target_msg_.velocity[i] = command_vel * kDegToRad;
        target_msg_.effort[i] = command_eff;
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
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Gets the reactive counterweight torque bounds in use (Nm), as
 *        {lower, upper}.
 */
std::pair<double, double> HebiThread::GetReactiveCwBounds() const {
    return {reactive_cw_lower_bound_, reactive_cw_upper_bound_};
}

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main command loop.
 *
 *        Delegates torque checks, movement (trajectory/dynamics), and feedback
 *        publishing to helper functions for readability.
 *
 * @see ExecuteMovement SendFeedback
 */
void HebiThread::run() {
    logger_->Debug("Starting thread");

    // Initialize buffers for loop efficiency
    Eigen::VectorXd cmd_pos = Eigen::VectorXd::Zero(n_actuators_);
    Eigen::VectorXd cmd_vel = Eigen::VectorXd::Zero(n_actuators_);
    Eigen::VectorXd cmd_acc = Eigen::VectorXd::Zero(n_actuators_);  // <- traj
    Eigen::VectorXd cmd_trq = Eigen::VectorXd::Zero(n_actuators_);  // -> cmd

    // For dynamic compensation (DynamicComp)
    auto last_loop_time = std::chrono::steady_clock::now();

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        if (group_ == nullptr) {
            emit ReportTelemetry(GetTelemetry(), type_);  // "Not Connected"
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
        const bool command_sent = ExecuteMovement(dt, cmd_pos, cmd_vel, cmd_acc,
                                                  cmd_trq);
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
    SetMaxVel(*command_, "./bin/shared/hebi/safety.xml");

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

    // Same pose for loop's position hold; reset clock
    latest_cmd_pos_ = feedback_->getPosition();
    latest_cmd_vel_ = Eigen::VectorXd::Zero(n_actuators_);
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        t_trajectory_s_ = 0.0;
    }

#if LIBRA_VERSION == 2
    motion_controller_->Reset();
    for (const auto& joint : motion_controller_->GetJoints()) {
        bool driven = false;
        for (int i = 0; i < n_actuators_; ++i) {
            driven = driven || (JointFromActuator(enum_names_[i]) == joint);
        }

        if (!driven) {
            emit ErrorThrown(
                "HEBI - No connected actuator drives the "
                + QString::fromStdString(Joint::NameEnumToString(joint))
                + " joint; motion controller commands will be ignored!");
        }
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
        ReportCalibrationStatus();
        pitch_released_ = false;
        emit PitchReleaseChanged(false);
#endif

        // Destructing hebi::Group automatically cleans it up
        group_.reset();
        command_->clear();

        ReportMovement(false);  // control loop inactive

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
    if (pitch_released_) {
        // A "move" may still command the EPOS yaw joint, so don't error
        logger_->Warn("Ignoring target while pitch is released");
        return;
    }
#endif

    // Prepare targets and begin motion
    const Eigen::VectorXd target_pos =
        Eigen::Map<const Eigen::VectorXd>(target.data(), target.size())
        * kDegToRad;

#if LIBRA_VERSION == 2
    if (Motion::UsesMotionController(motion_mode_)) {
        // Motion controller plans the move and requests it (in joint space,
        // since it relies on specific kinematics)
        std::vector<double> target_joint(Motion::kNumJoints, kNone);
        for (int i = 0; i < n_actuators_; ++i) {
            const auto joint = JointFromActuator(enum_names_[i]);
            if (joint != Joint::Name::kUndefined) {
                target_joint[joint] = Joint::ActuatorToJointSimple(joint,
                                                                   target_pos(
                                                                       i));
            }
        }

        motion_controller_->SetTarget(target_joint);
        move_requested_ = true;
        return;
    }

    // New target overrides a running calibration
    if (calibration_running_) {
        motion_controller_->CancelCalibration();
    }
#endif

    // Control loop starts it from the held pose
    // NOTE: Original LIBRA-I code started from measured pose
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        pending_target_pos_ = target_pos;
    }
    move_requested_ = true;
}

/**
 * @brief Halts the trajectories of all actuators.
 */
void HebiThread::Stop() {
    if (group_ == nullptr) {
        return;  // do nothing
    }

    // End trajectory following; the arm holds its last commanded pose
    move_requested_ = false;
    {
        const std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_.reset();
        t_trajectory_s_ = 0.0;
        pending_target_pos_.reset();
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
 * @brief Loads new safety limits for the connected actuators.
 *
 * @param file_path Path to an XML safety parameters file
 *
 * @note The file's velocity limit (if any) also becomes the max. trajectory
 *       speed, from the next trajectory onward.
 */
void HebiThread::LoadSafety(const QString& file_path) {
    if (group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't load safety limits; not connected!");
        return;
    }

    hebi::GroupCommand safety_cmd(group_->size());
    if (!safety_cmd.readSafetyParameters(file_path.toStdString())) {
        emit ErrorThrown("HEBI - Failed to load safety limits from: "
                         + file_path);
        return;
    }

    if (!group_->sendCommandWithAcknowledgement(safety_cmd, kTimeout)) {
        emit ErrorThrown(
            "HEBI - Didn't receive acknowledgement from safety update!");
        return;
    }
    SetMaxVel(safety_cmd, file_path.toStdString());

    logger_->Info("Successfully loaded safety limits from: "
                  + file_path.toStdString());
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
 * @brief Sets the motion mode the control loop follows.
 *
 * @param mode Mode chosen by MainWindow, which owns it and configures the
 *             motion controller to match
 */
void HebiThread::SetMotionMode(const Motion::MotionMode& mode) {
    if (mode == Motion::MotionMode::kManual) {
        // Without reactive CW, movement is always allowed
        movement_en_ = true;
        trajectory_rate_ = 1.0;  // full speed
    }

    motion_mode_ = mode;
}

/**
 * @brief Sets the central joint's reactive counterweight torque bounds.
 *
 * @param lower_bound Lower torque bound (Nm)
 * @param upper_bound Upper torque bound (Nm)
 */
void HebiThread::SetReactiveCwBounds(const double& lower_bound,
                                     const double& upper_bound) {
#if LIBRA_VERSION == 2
    // TODO: bounds should be set in one place and referenced when needed
    motion_controller_->SetReactiveCwBounds(lower_bound, upper_bound);
#endif

    reactive_cw_lower_bound_ = lower_bound;
    reactive_cw_upper_bound_ = upper_bound;
}

#if LIBRA_VERSION == 2
/**
 * @brief Releases or takes control of the pitch joint.
 *
 * @param enabled Whether to stop commanding the actuator
 *
 * @note Sets the control strategy to "Off" instead of just sending nothing:
 *       HEBI's safety controllers (e.g., velocity limit) still act on an
 *       uncommanded actuator, resisting back-driving. The previous strategy is
 *       restored when taking control again.
 */
void HebiThread::EnablePitchRelease(const bool& enabled) {
    if (enabled && group_ == nullptr) {
        emit ErrorThrown("HEBI - Can't release pitch; not connected!");
        emit PitchReleaseChanged(false);
        return;
    }

    if (enabled && !pitch_released_) {
        // Save current strategies to restore later (same order in both enums)
        hebi::GroupInfo info(group_->size());
        const bool info_received = group_->requestInfo(info, kTimeout);
        strategies_before_release_.clear();
        for (size_t i = 0; info_received && i < group_->size(); ++i) {
            const auto& strategy =
                info[i].settings().actuator().controlStrategy();
            if (!strategy) {
                break;
            }
            strategies_before_release_.push_back(
                static_cast<hebi::Command::ControlStrategy>(strategy.get()));
        }
        if (strategies_before_release_.size() != group_->size()) {
            emit ErrorThrown("HEBI - Can't release pitch; actuator didn't "
                             "report its control strategy!");
            emit PitchReleaseChanged(false);
            return;
        }

        Stop();
        motion_controller_->Reset();
        pitch_released_ = true;  // control loop stops sending commands

        hebi::GroupCommand off_cmd(group_->size());
        for (size_t i = 0; i < group_->size(); ++i) {
            off_cmd[i].settings().actuator().controlStrategy().set(
                hebi::Command::ControlStrategy::Off);
        }
        if (!group_->sendCommandWithAcknowledgement(off_cmd, kTimeout)) {
            emit ErrorThrown("HEBI - Didn't receive acknowledgement from "
                             "control strategy change (Off)!");
        }
        logger_->Info("Pitch released (control strategy Off)");

    } else if (!enabled && pitch_released_) {
        hebi::GroupCommand restore_cmd(group_->size());
        for (size_t i = 0; i < strategies_before_release_.size(); ++i) {
            restore_cmd[i].settings().actuator().controlStrategy().set(
                strategies_before_release_[i]);
        }
        if (!group_->sendCommandWithAcknowledgement(restore_cmd, kTimeout)) {
            emit ErrorThrown("HEBI - Didn't receive acknowledgement from "
                             "control strategy change (restore)!");
        }

        pitch_released_ = false;  // control loop holds current pose again
        logger_->Info("Pitch controlled (holding current pose)");
    }

    emit PitchReleaseChanged(pitch_released_);
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
 * @note Sweep speeds are fractions of `kMaxSweepVel` (or `max_vel_`, if
 *       lower).
 */
void HebiThread::StartFrictionCalibration(const double& deg_a,
                                          const double& deg_b) {
    if (!CanCalibrate()) {
        return;
    }
    if (!motion_controller_
             ->StartFrictionCalibration(deg_a * kDegToRad, deg_b * kDegToRad,
                                        std::min(kMaxSweepVel,
                                                 max_vel_.load()))) {
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

/**
 * @brief Turns the periodic log of the motion controller's model values on or
 *        off.
 *
 * @param enabled Whether to log the model values
 */
void HebiThread::EnableModelLogging(const bool& enabled) {
    diag_log_en_ = enabled;
    logger_->Info("Model value logging "
                  + std::string(enabled ? "enabled" : "disabled"));
}
#endif
