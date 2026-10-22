/******************************************************************************
 * @file   abstract_motion_controller.h
 * @brief  Abstract class for system-kinematics-informed motion controllers;
 *         header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// Other Library Headers
// (none)

// Project Headers
#include "joint_defs.h"
#include "logger.h"
#include "motion_defs.h"

/**
 * @brief Provides common interfaces for inherently different motion control
 *        strategies.
 *
 * @note There is no virtual `Update()` function: LIBRA-I and LIBRA-II are
 *       mechanically different, so each motion controller defines its own.
 *
 * @note All public functions must lock `mutex_` to ensure thread safety.
 */
class AbstractMotionController {
  public:
    explicit AbstractMotionController(const std::vector<Joint::Name>& joints,
                                      const bool& debug_mode);
    virtual ~AbstractMotionController() = default;

    AbstractMotionController(const AbstractMotionController&) = delete;
    AbstractMotionController& operator=(const AbstractMotionController&) =
        delete;

    [[nodiscard]] const std::vector<Joint::Name>& GetJoints() const {
        return joints_;
    }

    void SetDebugMode(const bool& enabled);

    // --- Commands ---

    void SetTarget(const std::vector<double>& target_pos);
    void Stop();
    void Reset();

  protected:
    // --- Hooks (called with `mutex_` locked) ---

    virtual void OnTargetChanged() {}

    virtual void OnStop() {}

    virtual void OnReset() {}

    // --- Helper Functions ---

    [[nodiscard]] bool IsValidInput(
        const Motion::ActualState& actual,
        const Motion::CommandedState& commanded) const;

    static std::string Format(const double& value);

    // --- Data Members ---

    std::unique_ptr<Logger> logger_;
    const std::vector<Joint::Name> joints_;  // joints driven by this controller

    mutable std::mutex mutex_;

    std::optional<std::vector<double>> target_pos_;
};
