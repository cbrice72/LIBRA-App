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
#include "robot_model.hpp"     // HEBI
#include "trajectory.hpp"      // HEBI

// Project Headers
#include "abstract_actuator.h"

/**
 * @brief Control class for HEBI actuators.
 *
 * @see abstract_actuator.h
 */
class HebiThread : public AbstractActuatorThread {
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

    // --- Data Members ---

    std::vector<std::string> families_;
    std::vector<std::string> names_;

    hebi::robot_model::RobotModel model_;

    std::shared_ptr<hebi::Group> group_;
    const int num_actuators_;  // set in constructor initializer list

    std::shared_ptr<hebi::GroupCommand> command_;
    std::shared_ptr<hebi::GroupFeedback> feedback_;
    std::vector<Actuator::Joint> joint_order_;  // NOTE: should match `names_`

    std::shared_ptr<hebi::trajectory::Trajectory> trajectory_;
    std::chrono::time_point<std::chrono::system_clock> trajectory_start_time_;
};
