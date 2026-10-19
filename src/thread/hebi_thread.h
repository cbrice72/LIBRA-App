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

// Forward Declarations
#if LIBRA_VERSION == 2
namespace Motion {
class Libra2MotionController;  // see libra2_motion_controller.h
struct SweepRequest;           // "
}  // namespace Motion
#endif

#ifdef BUILD_WITH_ROS2
using msgJointState = sensor_msgs::msg::JointState;
#endif

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
    void EnableFreeMode(const bool& enabled);
    void EnableInertiaFeedforward(const bool& enabled);
    void SetHandoffBand(const double& deg);

    void UpdateCounterweightVolume(const double& volume);
    void UpdateCounterweightFlow(const double& flow);

    // --- Calibration ---

    void StartLevelCalibration();
    void StartTwoAngleCalibration(const double& deg_a, const double& deg_b);
    void StartFrictionCalibration(const double& deg_a, const double& deg_b);
    void CancelCalibration();

    void SaveCalibration(const QString& file_path, const double& extension,
                         const QString& note);
    void LoadCalibration(const QString& file_path);
#endif

  signals:
    // --- Actuator Updates ---

    // NOTE: See AbstractActuatorThread for generic signals

    void ReportArmTorque(const std::optional<double>& theta);
#if LIBRA_VERSION == 1
    void InformPitch(const double& angle);
#elif LIBRA_VERSION == 2
    void MotionModeChanged(const bool& cw_primary);
    void FreeModeChanged(const bool& enabled);
    void MovementStateChanged(const bool& moving);
    void CalibrationFinished(const bool& success, const bool& arm_calibrated);
    void CalibrationLoaded(const bool& arm_calibrated);
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
    void StartSweep(const Motion::SweepRequest& sweep);
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

    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_;
    std::mutex trajectory_mutex_;

    double t_trajectory_s_{0.0};  // s, advances by dt * playback_rate_ / cycle

    bool time_unit_warned_{false};  // see `StartTrajectory()`

#if LIBRA_VERSION == 1
    // Enables logic that uses torque feedback hysteresis to control trajectory
    // movement state (see `movement_en_`)
    bool torque_control_en_{false};
    double torque_comp_lower_bound_{kTorqueCompLowerBound};  // Nm
    double torque_comp_upper_bound_{kTorqueCompUpperBound};  // Nm

    // Enables trajectory-based movement
    bool movement_en_{true};

    double playback_rate_{1.0};  // 0.5 in the soft torque band
#elif LIBRA_VERSION == 2
    std::unique_ptr<Motion::Libra2MotionController> motion_controller_;
    std::atomic<bool> free_mode_{false};

    // Last position/velocity command sent (actuator space)
    Eigen::VectorXd pos_cmd_;  // rad
    Eigen::VectorXd vel_cmd_;  // rad/s

    // Report-only state, so the UI hears about each change once
    std::atomic<bool> moving_{false};
    std::atomic<bool> calibration_running_{false};
    double t_diag_log_elapsed_s_{0.0};  // s since last log (for throttling)
#endif

#ifdef BUILD_WITH_ROS2
    rclcpp::Publisher<msgJointState>::SharedPtr state_pub_;
    rclcpp::Publisher<msgJointState>::SharedPtr target_pub_;
    msgJointState state_msg_;  // reused for efficiency
    msgJointState target_msg_;
#endif
};
