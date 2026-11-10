/******************************************************************************
 * @file   hebi_thread.h
 * @brief  Control class for LIBRA HEBI actuators; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// Other Library Headers
#include "Eigen/Core"          // Eigen
#include "group.hpp"           // HEBI
#include "group_command.hpp"   // HEBI
#include "group_feedback.hpp"  // HEBI
#include "robot_model.hpp"     // HEBI
#include "trajectory.hpp"      // HEBI
#ifdef BUILD_WITH_ROS2
# include <rclcpp/rclcpp.hpp>                // ROS2 Core
# include <sensor_msgs/msg/joint_state.hpp>  // ROS2 Messages
#endif

// Project Headers
#include "abstract_actuator_thread.h"
#include "motion_defs.h"
#if LIBRA_VERSION == 2
# include "libra2_motion_controller.h"
#endif

#ifdef BUILD_WITH_ROS2
using msgJointState = sensor_msgs::msg::JointState;
#endif

// Torque compensation constants
constexpr double kTorqueCompLowerBound = 3.0;  // Nm
constexpr double kTorqueCompUpperBound = 6.0;  // Nm
constexpr double kTorqueCompSlowRate = 0.5;    // velocity rate between bounds

/**
 * @brief Control class for HEBI actuators.
 *
 * @see abstract_actuator_thread
 */
class HebiThread : public AbstractActuatorThread
#ifdef BUILD_WITH_ROS2
    ,
                   public rclcpp::Node
#endif
{
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit HebiThread(QObject* parent,
#if LIBRA_VERSION == 2
                        std::shared_ptr<Libra2MotionController> motion_controller,
#endif
                        const bool& debug_mode = false);
    explicit HebiThread(
        QObject* parent,
#if LIBRA_VERSION == 2
        std::shared_ptr<Libra2MotionController> motion_controller,
#endif
        std::vector<std::string> families, std::vector<std::string> names,
        const bool& debug_mode = false);
    ~HebiThread() override;

  public slots:
    // --- Actuator Commands ---

    void Connect() override;
    void Disconnect() override;

    void SetTarget(const std::vector<double>& deg) override;
    void Stop() override;

    void LoadGains(const QString& file_path);
    void LoadSafety(const QString& file_path);
    void StartHebiLog();
    void StopHebiLog();

    // --- Control Algorithm Settings ---

    void SetMotionMode(const Motion::MotionMode& mode);
    void SetTorqueCompBounds(const double& lower, const double& upper);

#if LIBRA_VERSION == 2
    void EnableFreeMode(const bool& enabled);
    void EnableInertiaFeedforward(const bool& enabled);
    void SetHandoffBand(const double& deg);

    // --- Calibration ---

    void StartLevelCalibration();
    void StartTwoAngleCalibration(const double& deg_a, const double& deg_b);
    void StartFrictionCalibration(const double& deg_a, const double& deg_b);
    void CancelCalibration();

    void EnableModelLogging(const bool& enabled);
#endif

  signals:
    // --- Actuator Updates ---

    // NOTE: See AbstractActuatorThread for generic signals

    void ReportArmTorque(const std::optional<double>& theta);
#if LIBRA_VERSION == 1
    void InformPitch(const double& angle);
#elif LIBRA_VERSION == 2
    void FreeModeChanged(const bool& enabled);

    void CalibrationChanged(const Motion::Calibration& level);
    void CalibrationProgressChanged(const QString& text, const bool& running);
#endif

  protected:
    void run() override;

  private:
    // --- Helper Functions ---

    std::unordered_map<Joint::Name, double> GetJointFeedbackMap(
        const std::shared_ptr<hebi::GroupFeedback>& feedback,
        const Actuator::Feedback& feedback_type);

    bool ExecuteMovement(std::chrono::duration<double> dt,
                         Eigen::VectorXd& cmd_pos, Eigen::VectorXd& cmd_vel,
                         Eigen::VectorXd& cmd_acc, Eigen::VectorXd& cmd_trq);
    double UpdateTrajectoryRate();
    void StartTrajectory(const Eigen::VectorXd& start_pos,
                         const Eigen::VectorXd& target_pos);
    [[nodiscard]] Eigen::VectorXd HeldPosition() const;
    bool FollowTrajectory(const double& dt_s, Eigen::VectorXd& cmd_pos,
                          Eigen::VectorXd& cmd_vel, Eigen::VectorXd& cmd_acc,
                          const double& trajectory_rate);
#if LIBRA_VERSION == 2
    std::optional<Motion::MotionCommand> RunMotionController(
        const double& dt_s, Eigen::VectorXd& cmd_vel, Eigen::VectorXd& cmd_acc);
    void StartSweep(const Motion::SweepRequest& sweep);
    void ReportCalibrationStatus();

    bool CanCalibrate();
#endif

    void SetMaxVel(const hebi::GroupCommand& safety_cmd,
                   const std::string& source);

    void SendFeedback();
    [[nodiscard]] QString GetTelemetry() const;
#ifdef BUILD_WITH_ROS2
    void PublishState();
#endif

    // --- Data Members ---

    std::vector<std::string> families_;
    std::vector<std::string> names_;
    std::vector<Actuator::Name> enum_names_;
    const int n_actuators_;
    const unsigned long loop_period_ms_;

    std::shared_ptr<hebi::Group> group_;
    std::shared_ptr<hebi::GroupCommand> command_;
    std::shared_ptr<hebi::GroupFeedback> feedback_;

    // clang-format off
    Eigen::IOFormat matrix_print_format_{Eigen::StreamPrecision,
                                         Eigen::DontAlignCols,
                                         ", ", ", ", "", "", "", ""};
    // clang-format on

    bool logging_active_{false};

    // Improve performance of GetTelemetry() by allocating these vectors here
    // (the `mutable` keyword allows a const function to modify class members)
    mutable std::vector<double> a_vel_;     // actual velocity (deg/s)
    mutable std::vector<double> defl_;      // deflection (deg)
    mutable std::vector<double> defl_vel_;  // deflection velocity (deg/s)
    mutable std::vector<double> volt_;      // voltage (V)
    mutable std::vector<double> curr_;      // motor current (A)
    mutable std::vector<double> temp_;      // motor winding temperature (C)

    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_;
    std::mutex trajectory_mutex_;

    double t_trajectory_s_{0.0};  // s, incr. by dt * trajectory_rate_ / cycle

    std::atomic<double> max_vel_;  // peak trajectory speed (rad/s)

    // From `SetTarget()`, started by the control loop (in LIBRA-II: only used
    // by modes without the motion controller; guarded by `trajectory_mutex_`)
    std::optional<Eigen::VectorXd> pending_target_pos_;  // rad

    // Latest position/velocity command (actuator space)
    Eigen::VectorXd latest_cmd_pos_;  // rad
    Eigen::VectorXd latest_cmd_vel_;  // rad/s

    // Owned by MainWindow (see `SetMotionMode()`)
    std::atomic<Motion::MotionMode> motion_mode_{Motion::MotionMode::kManual};

    // Used by ATC (see `UpdateTrajectoryRate()`)
    double torque_comp_lower_bound_{kTorqueCompLowerBound};  // Nm
    double torque_comp_upper_bound_{kTorqueCompUpperBound};  // Nm
    bool movement_en_{true};       // false = trajectory paused
    double trajectory_rate_{1.0};  // 0.5 in the lower torque band

#if LIBRA_VERSION == 2
    std::shared_ptr<Libra2MotionController> motion_controller_;
    std::atomic<bool> free_mode_{false};

    // Report-only state, so the UI hears about each change once
    std::atomic<bool> calibration_running_{false};
    std::atomic<Motion::Calibration> calibration_level_{
        Motion::Calibration::kNone};
    std::string last_progress_;  // last calibration step sent to the UI

    std::atomic<bool> diag_log_en_{false};  // periodic model log
    double t_diag_log_elapsed_s_{0.0};      // s since last log (for throttling)
#endif

#ifdef BUILD_WITH_ROS2
    rclcpp::Publisher<msgJointState>::SharedPtr state_pub_;
    rclcpp::Publisher<msgJointState>::SharedPtr target_pub_;
    msgJointState state_msg_;  // reused for efficiency
    msgJointState target_msg_;
#endif
};
