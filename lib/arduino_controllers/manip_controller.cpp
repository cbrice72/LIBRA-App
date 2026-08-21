/******************************************************************************
 * @file   manip_controller.cpp
 * @brief  Manip Arduino (tip servos) device management; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "manip_controller.h"

// C++ Standard Library Headers
// (none)

// Other Library Headers
// (none)

// Project Headers
// (none)

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Class Helpers
 * !Arduino Commands (slots)
 */

// Convenience Indexes for Individual Manipulator Control

constexpr uint8_t kPitch = 0;
constexpr uint8_t kPan = 1;
constexpr uint8_t kTilt = 2;

// Control Parameters

// NOTE: The original code used 90.0 / (60.0 * 60.0) = 0.025 deg per update which,
//       likely running at 60 Hz, gives 1.5 deg/s. Since that was a nice speed,
//       we use known constants to recreate it here.
constexpr double kManipSlowSpeed = 1.5;  // deg/s
constexpr double kManipUpdateRate =
    1000.0 / 500.0;  // Hz (Assuming 500ms update interval)
constexpr double kManipSlowMultiplier = kManipSlowSpeed / kManipUpdateRate;

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param debug_mode Whether to output verbose debug text
 */
ManipController::ManipController(QObject* parent, const bool& debug_mode)
    : GenericSerialDevice(parent, debug_mode, "Manip") {}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Sends the current command to the ManipArduino.
 */
void ManipController::OnUpdate() {
    // Slow movement calculation
    for (auto i = 1; i < 3; ++i) {
        // Manually calculate steps to achieve a slow pace
        // (NOTE: Since the manipulator's pitch (i = 0) is adjusted automatically
        //        based on the arm's pitch, this loop starts at i = 1)
        if (slow_direction_.at(i) != 0) {
            // Increase by small arbitrary amount
            current_pos_.at(i) += slow_direction_.at(i) * kManipSlowMultiplier;

            // If target has been reached, reset relevant variables
            /*
            // NOTE: There is no check for moving in a "negative"
            //       direction because the servos do not take negative
            //       values. This conversion is handled in the file
            //       `arduino/servo_arduino/servo_arduino.ino`, function
            //       `mapfloat()`.
            if ((current_pos_.at(i) > target_pos_.at(i))
                && (slow_direction_.at(i) == 1)) {
            */
            if ((slow_direction_.at(i) == 1
                 && current_pos_.at(i) > target_pos_.at(i))
                || (slow_direction_.at(i) == -1
                    && current_pos_.at(i) < target_pos_.at(i))) {
                current_pos_.at(i) = target_pos_.at(i);
                slow_direction_.at(i) = 0;
            }
        }
    }

    // Prepare and send command: "<PITCH> <PAN> <TILT>\n"
    QString servo_cmd = QString::asprintf("%.1f %.1f %.1f\n",
                                          current_pos_.at(kPitch),
                                          current_pos_.at(kPan),
                                          current_pos_.at(kTilt));

    if (serial_port_->write(servo_cmd.toUtf8()) == -1) {
        throw std::runtime_error("Failed to write to port");
    }

    emit ReportPosition(current_pos_.at(kPitch), current_pos_.at(kPan),
                        current_pos_.at(kTilt));
}

//------------------------------------------------------------------------------
// !Arduino Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Sets the state of the manipulator pitch correction functionality.
 *
 * @param enabled Whether to enable pitch correction
 */
void ManipController::EnablePitchCorrection(const bool& enabled) {
    logger_->Debug(std::string(enabled ? "Enabling" : "Disabling")
                   + " auto-correction for arm pitch");

    manip_correction_enabled_ = enabled;
}

/**
 * @brief Sets correction target for manipulator servo #1.
 *
 * @param pitch Angle of LIBRA-I arm pitch joint "J3"
 */
void ManipController::UpdatePitchFeedback(const double& pitch) {
    if (!serial_port_->isOpen()) {
        // NOTE: Unlike other isOpen checks, do NOT log a message here because
        //       this function can be called frequently by HebiThread
        return;
    }

    // Do nothing if disabled
    if (!manip_correction_enabled_) {
        return;
    }

    current_pos_.at(kPitch) = (pitch <= 0) ? -pitch : 180 - pitch;
}

/**
 * @brief Sets movement targets for manipulator servos #2 and #3.
 *
 * @param pan Target yaw angle
 * @param tilt Target pitch angle
 * @param move_slow Whether to use a slower, more controlled trajectory
 */
void ManipController::SetTarget(const double& pan, const double& tilt,
                                const bool& move_slow) {
    if (!serial_port_->isOpen()) {
        logger_->Error("Cannot set targets; not connected");
        return;
    }

    target_pos_.at(kPan) = pan;
    target_pos_.at(kTilt) = -tilt;  // "+" should rotate "upwards"

    if (move_slow) {
        // Only calculate the direction (position is set in UpdateManip)
        auto get_direction = [](double target, double current) {
            return (target > current) ? 1 : (target < current) ? -1 : 0;
        };

        slow_direction_.at(kPan) = get_direction(pan, current_pos_.at(kPan));
        slow_direction_.at(kTilt) = get_direction(tilt, current_pos_.at(kTilt));
    } else {
        // Ensure slow mode is disabled
        slow_direction_.at(kPan) = 0;
        slow_direction_.at(kTilt) = 0;

        // NOTE: Setting the commanded positions to the target values causes the
        //       servos to move at maximum speed (near-instant)
        current_pos_.at(kPan) = pan;
        current_pos_.at(kTilt) = tilt;
    }

    logger_->Debug("Set targets to " + std::to_string(pan) + " "
                   + std::to_string(tilt) + " deg");
}
