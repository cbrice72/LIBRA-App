/******************************************************************************
 * @file   abstract_motion_controller.cpp
 * @brief  Abstract class for system-kinematics-informed motion controllers;
 *         implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "abstract_motion_controller.h"

// C++ Standard Library Headers
#include <cmath>
#include <iomanip>
#include <sstream>

// Project Headers
#include "qt_logger.h"

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Class Helpers
 * !Commands
 */

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param joints Joints to be driven by the motion controller
 * @param debug_mode Whether to output verbose debug text
 */
AbstractMotionController::AbstractMotionController(
    const std::vector<Joint::Name>& joints, const bool& debug_mode)
    : joints_(joints) {
    logger_ = std::make_unique<QtLogger>("motion_controller", debug_mode);
    logger_->Debug("Initializing motion controller");
}

/**
 * @brief Sets whether debug messages are logged.
 *
 * @param enabled Whether to output verbose debug text
 */
void AbstractMotionController::SetDebugMode(const bool& enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    logger_->SetDebugMode(enabled);
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Checks that every input array has one finite entry per driven joint
 *        (others may be NaN).
 *
 * @param actual Actual joint values to check
 * @param commanded Commanded joint values to check
 */
bool AbstractMotionController::IsValidInput(
    const Motion::ActualState& actual,
    const Motion::CommandedState& commanded) const {
    // Check container sizes
    bool valid = actual.pos.size() == Motion::kNumJoints
                 && actual.vel.size() == Motion::kNumJoints
                 && actual.torque.size() == Motion::kNumJoints
                 && commanded.vel.size() == Motion::kNumJoints
                 && commanded.acc.size() == Motion::kNumJoints;

    // Check validity of values
    for (const auto& joint : joints_) {
        valid = valid && joint < Motion::kNumJoints
                && std::isfinite(actual.pos[joint])
                && std::isfinite(actual.vel[joint])
                && std::isfinite(actual.torque[joint])
                && std::isfinite(commanded.vel[joint])
                && std::isfinite(commanded.acc[joint]);
    }

    return valid;
}

/**
 * @brief Prettifies a number for command-line output.
 *
 * @param value Number to format
 */
std::string AbstractMotionController::Format(const double& value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << value;
    return oss.str();
}

//------------------------------------------------------------------------------
// !Commands
//------------------------------------------------------------------------------

/**
 * @brief Sets the pose to reach and hold.
 *
 * @param target_pos One angle per driven joint, indexed by `Joint::Name` (rad)
 */
void AbstractMotionController::SetTarget(const std::vector<double>& target_pos) {
    std::lock_guard<std::mutex> lock(mutex_);

    bool valid = (target_pos.size() == Motion::kNumJoints);
    for (const auto& joint : joints_) {
        valid = valid && joint < Motion::kNumJoints
                && std::isfinite(target_pos[joint]);
    }
    if (!valid) {
        logger_->Warn("Ignoring invalid target (expected "
                      + std::to_string(Motion::kNumJoints)
                      + " angles, finite for every driven joint)");
        return;
    }

    target_pos_ = target_pos;
    OnTargetChanged();  // implemented by child
}

/**
 * @brief Clears the target and halts movement.
 */
void AbstractMotionController::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    target_pos_.reset();
    OnStop();  // implemented by child
}

/**
 * @brief Clears runtime state (keeps settings and calibration); call on
 *        disconnect.
 */
void AbstractMotionController::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    target_pos_.reset();
    OnReset();  // implemented by child
}
