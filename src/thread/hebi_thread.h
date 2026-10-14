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

// Forward Declarations
#if LIBRA_VERSION == 2
namespace Motion {
class Libra2MotionController;  // see libra2_motion_controller.h
}  // namespace Motion
#endif

// TODO: BUG - this alias is outside the BUILD_WITH_ROS2 guard, so standalone
//       builds (BUILD_WITH_ROS2=OFF) fail to compile
using msgJointState = sensor_msgs::msg::JointState;

// Torque compensation constants
constexpr double kTorqueCompLowerBound = 3.0;  // Nm
constexpr double kTorqueCompUpperBound = 6.0;  // Nm

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
    explicit HebiThread(QObject* parent, const bool& debug_mode = false);
    explicit HebiThread(QObject* parent, std::vector<std::string> families,
                        std::vector<std::string> names,
                        const bool& debug_mode = false);
    ~HebiThread() override;

  public slots:
    // --- Actuator Commands ---

    void Connect() override;
    void Disconnect() override;

    void SetTarget(const std::vector<double>& deg) override;
    void Stop() override;

    void LoadGains(const QString& file_path);
    void StartHebiLog();
    void StopHebiLog();

    // --- Control Algorithm Settings ---

    void EnableTorqueControl(const bool& enabled);
    void SetTorqueCompBounds(const double& lower, const double& upper);

#if LIBRA_VERSION == 2
    void EnableCwPrimaryMode(const bool& enabled);
    void SetHandoffBand(const double& deg);
    void UpdateCounterweightVolume(const double& liters);

    // --- Calibration ---

    void StartLevelCalibration();
    void StartTwoAngleCalibration(const double& deg_a, const double& deg_b);
    void CancelCalibration();
#endif

  signals:
    // --- Actuator Updates ---

    // NOTE: See AbstractActuatorThread for generic signals

    void ReportArmTorque(const std::optional<double>& theta);
#if LIBRA_VERSION == 1
    void InformPitch(const double& angle);
#elif LIBRA_VERSION == 2
    void MotionModeChanged(const bool& cw_primary);
    void MovementStateChanged(const bool& moving);
    void CalibrationFinished(const bool& success);
#endif

  protected:
    void run() override;

  private:
    // --- Helper Functions ---

    std::unordered_map<Joint::Name, double> GetJointFeedbackMap(
        const std::shared_ptr<hebi::GroupFeedback>& feedback,
        const Actuator::Feedback& feedback_type);

    void StartTrajectory(const Eigen::VectorXd& start_pos,
                         const Eigen::VectorXd& target_pos);
#if LIBRA_VERSION == 1
    void CheckTorqueControl();
    bool ExecuteMovement(std::chrono::duration<double> dt,
                         Eigen::VectorXd& cmd_pos, Eigen::VectorXd& cmd_vel,
                         Eigen::VectorXd& cmd_acc, Eigen::VectorXd& cmd_eff);
#elif LIBRA_VERSION == 2
    bool ExecuteMotionController(std::chrono::duration<double> dt,
                                 Eigen::VectorXd& cmd_pos,
                                 Eigen::VectorXd& cmd_vel,
                                 Eigen::VectorXd& cmd_acc,
                                 Eigen::VectorXd& cmd_eff);
    void ReportMotionStatus();
#endif

    void SendFeedback();
    [[nodiscard]] QString GetStatus() const;
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

    // Improve performance of GetStatus() by allocating these vectors here
    // (the `mutable` keyword allows a const function to modify class members)
    mutable std::vector<double> status_a_vel_;
    mutable std::vector<double> status_defl_;
    mutable std::vector<double> status_defl_vel_;
    mutable std::vector<double> status_volt_;
    mutable std::vector<double> status_curr_;
    mutable std::vector<double> status_temp_;

#if LIBRA_VERSION == 1
    // Enables logic that uses torque feedback hysteresis to control trajectory
    // movement state (see `movement_en_`)
    bool torque_control_en_{false};
    double torque_comp_lower_bound_{kTorqueCompLowerBound};  // Nm
    double torque_comp_upper_bound_{kTorqueCompUpperBound};  // Nm

    // Enables trajectory-based movement
    bool movement_en_{true};
#endif
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_;
    bool time_unit_warned_{false};  // see `StartTrajectory()`
#if LIBRA_VERSION == 1
    std::chrono::time_point<std::chrono::steady_clock> t_trajectory_start_{
        std::chrono::steady_clock::now()};
    std::chrono::steady_clock::duration t_trajectory_elapsed_{0};

    double vel_scale_{1.0};  // used when soft torque limit is exceeded
#elif LIBRA_VERSION == 2
    // Torque limits, movement enable, and playback speed live in the motion
    // controller; HebiThread only applies what it returns
    std::unique_ptr<Motion::Libra2MotionController> motion_controller_;

    // Trajectory clock (s); advances by dt * playback rate, so it freezes while
    // the motion controller pauses playback
    double t_trajectory_s_{0.0};

    // Last position/velocity command sent (actuator space). NaN = "no command"
    // (HEBI's convention), which is how a joint is left free to move.
    Eigen::VectorXd pos_cmd_;  // rad
    Eigen::VectorXd vel_cmd_;  // rad/s

    // Report-only state, so the UI hears about each change once
    std::atomic<bool> moving_{false};
    std::atomic<bool> calibration_running_{false};
    double t_diag_log_elapsed_s_{0.0};  // s since the last model log line
#endif

#ifdef BUILD_WITH_ROS2
    rclcpp::Publisher<msgJointState>::SharedPtr state_pub_;
    rclcpp::Publisher<msgJointState>::SharedPtr target_pub_;
    msgJointState state_msg_;  // reused for efficiency
    msgJointState target_msg_;
#endif
};
