/******************************************************************************
 * @file   abstract_motion_controller.cpp
 * @brief  Common motion controller base class implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "abstract_motion_controller.h"

// C++ Standard Library Headers
#include <cmath>
#include <iomanip>
#include <sstream>

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Commands
 * !Protected Functions
 */

namespace Motion {

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

/**
 * @brief Formats a number for log messages.
 *
 * @param value Number to format
 * @return std::string The number with a fixed number of decimals
 */
std::string MotionFormat(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);  // TODO: 2 decimals is a guess
    oss << value;
    return oss.str();
}

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param logger Logger to use (NOT owned; must outlive this object)
 * @param n_joints Number of HEBI-controlled joints (size of every array)
 */
AbstractMotionController::AbstractMotionController(Logger& logger,
                                                   std::size_t n_joints)
    : logger_(logger), n_joints_(n_joints) {}

//------------------------------------------------------------------------------
// !Commands
//------------------------------------------------------------------------------

/**
 * @brief Sets the target pose, to be reached and held.
 *
 * @param target_pos One angle per joint (rad, joint space)
 */
void AbstractMotionController::SetTarget(const JointValues& target_pos) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Reject anything the derived controller could not index safely
    bool valid = (target_pos.size() == n_joints_);
    for (double angle : target_pos) {
        valid = valid && std::isfinite(angle);
    }
    if (!valid) {
        logger_.Warn("Ignoring invalid target (expected "
                     + std::to_string(n_joints_) + " finite angle(s))");
        return;
    }

    target_pos_ = target_pos;
    OnTargetChanged();
}

/**
 * @brief Drops the target (the arm stays where it is).
 */
void AbstractMotionController::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    target_pos_.reset();
    OnStop();
}

/**
 * @brief Clears all runtime state but keeps settings and calibration. Call on
 *        disconnect.
 */
void AbstractMotionController::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    target_pos_.reset();
    OnReset();
}

//------------------------------------------------------------------------------
// !Protected Functions
//------------------------------------------------------------------------------

/**
 * @brief Checks that every array in a `MotionState` has one entry per joint.
 *
 * @param state State to check
 * @return true All sizes match `NumJoints()`
 * @return false At least one array has the wrong size
 */
bool AbstractMotionController::IsValid(const MotionState& state) const {
    return state.pos.size() == n_joints_ && state.vel.size() == n_joints_
           && state.torque_actuator.size() == n_joints_
           && state.ref_vel.size() == n_joints_
           && state.ref_acc.size() == n_joints_;
}

}  // namespace Motion
