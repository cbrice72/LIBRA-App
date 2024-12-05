/******************************************************************************
 * @file   hebi_actuator.h
 * @brief  Control class for LIBRA HEBI actuators; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <array>
#include <memory>
#include <string>
#include <vector>

// Other Library Headers
#include "group.hpp"           // HEBI
#include "group_command.hpp"   // HEBI
#include "group_feedback.hpp"  // HEBI

// Project Headers
#include "abstract_actuator.h"

/**
 * @brief Control class for HEBI actuators.
 *        Derived from `AbstractActuator` to provide a common interface.
 *
 * @note See `abstract_actuator.h`.
 */
class HebiActuator : public AbstractActuator {
  public:
    explicit HebiActuator(std::vector<std::string> families,
                          std::vector<std::string> names,
                          const bool& debug_mode = false);
    ~HebiActuator();

    // --- Actuator Commands ---

    bool Connect() override;
    bool Disconnect() override;

    void Move(double deg) override;
    void Stop() override;

    // --- Getters & Setters ---

    std::string GetStatus() override;

    double GetTargetPos() override;
    double GetActualPos() override;
    double GetActualTorque() override;

  private:
    // --- Helper Functions ---

    // --- Data Members ---

    std::vector<std::string> families_;
    std::vector<std::string> names_;

    std::shared_ptr<hebi::Group> group_;

    std::unique_ptr<hebi::GroupCommand> command_;
    std::unique_ptr<hebi::GroupFeedback> feedback_;
    // std::shared_ptr<hebi::trajectory::Trajectory> trajectory_{nullptr};
};
