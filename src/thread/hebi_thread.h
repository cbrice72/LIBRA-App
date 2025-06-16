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
#include <string>
#include <vector>

// Other Library Headers
#include "group.hpp"           // HEBI
#include "group_command.hpp"   // HEBI
#include "group_feedback.hpp"  // HEBI
#include "trajectory.hpp"      // HEBI
#ifdef BUILD_WITH_ROS2
# include <libra_app/msg/hebi_state.hpp>  // libra_interfaces
# include <rclcpp/rclcpp.hpp>             // ROS2 Core
#endif

// Project Headers
#include "abstract_actuator_thread.h"

typedef libra_app::msg::HebiState msgHebiState;

/**
 * @brief Control class for HEBI actuators.
 *
 * @see abstract_actuator_thread
 */
class HebiThread :
#ifdef BUILD_WITH_ROS2
    public rclcpp::Node,
#endif
    public AbstractActuatorThread {

  public:
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

  signals:
    // --- Actuator Updates ---

    // NOTE: see `AbstractActuatorThread`

  private:
    void run() override;

    // --- Helper Functions ---

    std::unordered_map<Actuator::Joint, double> GetFeedbackMap(
        const std::vector<double>& feedback);
    QString GetStatus();

#ifdef BUILD_WITH_ROS2
    void PublishState();
#endif

    // --- Data Members ---

    std::vector<std::string> families_;
    std::vector<std::string> names_;

    std::shared_ptr<hebi::Group> group_;
    const int num_actuators_;  // set in constructor initializer list

    std::shared_ptr<hebi::GroupCommand> command_;
    std::shared_ptr<hebi::GroupFeedback> feedback_;
    std::vector<Actuator::Joint> joint_order_;  // NOTE: should match `names_`

    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_;
    std::chrono::time_point<std::chrono::system_clock> trajectory_start_time_;

#ifdef BUILD_WITH_ROS2
    rclcpp::Publisher<msgHebiState>::SharedPtr state_pub_;
    msgHebiState state_msg_;  // reused for efficiency
#endif
};
