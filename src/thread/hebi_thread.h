/******************************************************************************
 * @file   hebi_thread.h
 * @brief  Control class for LIBRA HEBI actuators; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
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

  signals:
    // --- Actuator Updates ---

    // NOTE: See AbstractActuatorThread for generic signals

    void ReportArmTorque(const std::optional<double>& theta);
#if LIBRA_VERSION == 1
    void InformPitch(const double& angle);
#endif

  protected:
    void run() override;

  private:
    // --- Helper Functions ---

    std::unordered_map<Joint::Name, double> GetJointFeedbackMap(
        const std::shared_ptr<hebi::GroupFeedback>& feedback,
        const Actuator::Feedback& feedback_type);

    void CheckTorqueControl();
    void ExecuteMovement(std::chrono::duration<double> dt,
                         Eigen::VectorXd& cmd_pos, Eigen::VectorXd& cmd_vel,
                         Eigen::VectorXd& cmd_acc, Eigen::VectorXd& cmd_eff);

    void SendFeedback();
    [[nodiscard]] QString GetStatus() const;
#ifdef BUILD_WITH_ROS2
    void PublishState();
#endif

    // --- Data Members ---

    std::vector<std::string> families_;
    std::vector<std::string> names_;
    std::vector<Actuator::Name> enum_names_;
    const int n_actuators_;  // set in constructor initializer list

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

    // Enables logic that uses torque feedback hysteresis to control trajectory
    // movement state (see `movement_en_`)
    bool torque_control_en_{false};
    double torque_comp_lower_bound_{kTorqueCompLowerBound};  // Nm
    double torque_comp_upper_bound_{kTorqueCompUpperBound};  // Nm

    // Enables trajectory-based movement
    bool movement_en_{true};
    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_;
    std::chrono::time_point<std::chrono::steady_clock> trajectory_start_time_;

#ifdef BUILD_WITH_ROS2
    rclcpp::Publisher<msgJointState>::SharedPtr state_pub_;
    rclcpp::Publisher<msgJointState>::SharedPtr target_pub_;
    msgJointState state_msg_;  // reused for efficiency
    msgJointState target_msg_;
#endif
};
