/******************************************************************************
 * @file   water_controller.cpp
 * @brief  Water Arduino (counterweight system) device management;
 *         implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "water_controller.h"

// C++ Standard Library Headers
#include <bitset>
#include <cmath>

// Other Library Headers
// (none)

// Project Headers
// (none)

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Arduino Commands (slots)
 */

// Bit Positions for water_cmd_

constexpr uint8_t kWaterCmdMask = 0x0F;  // lower 4 bits
constexpr uint8_t kAIn = 0b1000;
constexpr uint8_t kBIn = 0b0100;
constexpr uint8_t kAOut = 0b0010;
constexpr uint8_t kBOut = 0b0001;

// NOLINTBEGIN(readability-identifier-naming)
// Angular Thresholds for Determining Direction of Torque

constexpr double k1_8Pi = M_PI / 8.0;
constexpr double k3_8Pi = M_PI * 3.0 / 8.0;
constexpr double k5_8Pi = M_PI * 5.0 / 8.0;
constexpr double k7_8Pi = M_PI * 7.0 / 8.0;

// NOLINTEND(readability-identifier-naming)

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Stringifies the lower 4 bits of a byte array for printing, since Qt
 *        doesn't seem to support this natively.
 *
 * @param bytes The QByteArray to convert
 * @return std::string String representation of the lower 4 bits
 */
std::string BytesToStr(const QByteArray& bytes) {
    std::string str("");

    for (auto byte : bytes) {
        str += std::bitset<4>(static_cast<unsigned char>(byte) & kWaterCmdMask)
                   .to_string();
    }

    return str;
}

}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param debug_mode Whether to output verbose debug text
 */
WaterController::WaterController(QObject* parent, const bool& debug_mode)
    : AbstractSerialDevice(parent, debug_mode, "Water") {
    water_cmd_.resize(1);  // only 4 bits needed, so reserve 1 byte
    ClearWaterCommand();
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Sends the current command to the SerialWater Arduino.
 */
void WaterController::OnUpdate() {
    qint64 bytes_written = serial_port_->write(water_cmd_);
    if (bytes_written == -1) {
        throw std::runtime_error("Failed to write to port");
    }

    SendWaterStatus(Water::Side::kA);  // emits ReportStatus
#if LIBRA_VERSION == 1
    SendWaterStatus(Water::Side::kB);
#endif
}

/**
 * @brief Clears the current water command.
 *
 * @param side The side of the fluid system to clear
 */
void WaterController::ClearWaterCommand(Water::Side side) {
    switch (side) {
        case Water::Side::kA:
            water_cmd_[0] &= static_cast<uint8_t>(~(kAIn | kAOut));
            break;
#if LIBRA_VERSION == 1
        case Water::Side::kB:
            water_cmd_[0] &= static_cast<uint8_t>(~(kBIn | kBOut));
            break;
#endif
        case Water::Side::kAll:
        default:
            water_cmd_[0] = 0;
            break;
    }
}

/**
 * @brief Modifies the current water command. Automatically enforces mutual
 *        exclusivity between commands to the same fluid system side.
 *
 * @param new_bits The bit(s) corresponding to the new command(s)
 *
 * @see kAIn kBIn kAOut kBOut
 */
void WaterController::ModifyWaterCommand(uint8_t new_bits) {
    new_bits &= kWaterCmdMask;  // ensure only lower 4 bits are used

    // Mutually exclusive: if a side's bit is modified, clear both bits first
    if ((new_bits & (kAIn | kAOut)) != 0) {
        ClearWaterCommand(Water::Side::kA);
    }
    if ((new_bits & (kBIn | kBOut)) != 0) {
#if LIBRA_VERSION == 1
        ClearWaterCommand(Water::Side::kB);
#else
        ClearWaterCommand(Water::Side::kAll);  // no kB if LIBRA_VERSION != 1
#endif
    }

    water_cmd_[0] |= static_cast<char>(new_bits);
}

/**
 * @brief Determines counterweight fill/drain command based on direction of
 *        torque acting upon central joint.
 *
 * @param torque_dir Direction of torque feedback, in radians (range: [-PI, PI])
 *
 * @note Command bit field "0b1234" -> 1: A_IN | 2: B_IN | 3: A_OUT | 4: B_OUT.
 *
 * @see hebi_thread::run
 */
void WaterController::MapTorqueToWaterCommand(const double& torque_dir) {
    if (!auto_comp_en_) {
        // Only allow manual control (see ForceCommand)
        return;
    }

    if (!serial_port_->isOpen()) {
        // NOTE: Unlike other isOpen checks, do NOT log a message here because
        //       this function can be called frequently by HebiThread
        return;
    }

    uint8_t command = 0;

#if LIBRA_VERSION == 1
    // Set command based on radial direction
    if (torque_dir > k7_8Pi || -k7_8Pi >= torque_dir) {
        command = kAIn | kBOut;  // 0b1001
    } else if (torque_dir > k5_8Pi) {
        command = kBOut;  // 0b0001
    } else if (torque_dir > k3_8Pi) {
        command = kAOut | kBOut;  // 0b0011
    } else if (torque_dir > k1_8Pi) {
        command = kAOut;  // 0b0010
    } else if (torque_dir > -k1_8Pi) {
        command = kBIn | kAOut;  // 0b0110
    } else if (torque_dir > -k3_8Pi) {
        command = kBIn;  // 0b0100
    } else if (torque_dir > -k5_8Pi) {
        command = kAIn | kBIn;  // 0b1100
    } else {
        command = kAIn;  // 0b1000
    }
#elif LIBRA_VERSION == 2
    // Set command regardless of which fluid system side is connected
    if (torque_dir == 0) {
        command = kAOut | kBOut;  // 0b0011
    } else {
        command = kAIn | kBIn;  // 0b1100
    }
#endif

    // If command is unchanged, do nothing
    if (command == (static_cast<uint8_t>(water_cmd_[0]) & kWaterCmdMask)) {
        return;
    }

    ModifyWaterCommand(command);

    logger_->Debug("[water]: Set command to " + BytesToStr(water_cmd_)
                   + " (A_IN | B_IN | A_OUT | B_OUT)");
}

/**
 * @brief Parses the active water command based on the specified side and
 *        reports its status.
 *
 * @param side The side of the fluid system to check
 */
void WaterController::SendWaterStatus(Water::Side side) {
    auto state = Water::State::kStopped;

    if (serial_port_->isOpen() && !water_cmd_.isEmpty()) {
        // Parse command to get corresponding state
        uint8_t command = static_cast<uint8_t>(water_cmd_[0]);
        switch (side) {
            case Water::Side::kA:
                if ((command & kAIn) != 0) {
                    state = Water::State::kFilling;
                } else if ((command & kAOut) != 0) {
                    state = Water::State::kDraining;
                }
                break;
#if LIBRA_VERSION == 1
            case Water::Side::kB:
                if ((command & kBIn) != 0) {
                    state = Water::State::kFilling;
                } else if ((command & kBOut) != 0) {
                    state = Water::State::kDraining;
                }
                break;
#endif
            default:
                // Do nothing (since state is initialized at the top)
                break;
        }
    }

    emit ReportStatus(side, state);
}

//------------------------------------------------------------------------------
// !Arduino Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Sets the state of the fluid system's automatic torque compensation.
 *
 * @param enabled Whether to enable auto-compensation
 *
 * @see MapTorqueToWaterCommand
 */
void WaterController::EnableAutoCompensation(const bool& enabled) {
    if (auto_comp_en_ != enabled) {
        ClearWaterCommand();  // reset the previous command when switching modes
    }
    auto_comp_en_ = enabled;

    logger_->Debug("[water]: ATC - Automatic torque compensation "
                   + std::string(enabled ? "enabled" : "disabled"));
}

/**
 * @brief Automatically updates the water command based on torque feedback.
 *        If no value is provided, the current command is cleared.
 *
 * @param torque_dir Direction of torque feedback, in radians (range: [-PI, PI])
 */
void WaterController::UpdateTorqueFeedback(
    const std::optional<double>& torque_dir) {
    if (!auto_comp_en_) {
        return;  // don't interfere with manual control
    }

    if (torque_dir.has_value()) {
        MapTorqueToWaterCommand(torque_dir.value());
    } else {
        ClearWaterCommand();
    }
}

/**
 * @brief Forces the state of a specific side of the fluid system.
 *
 * @param side The side to command
 * @param Water::State The state to force
 */
void WaterController::ForceCommand(const Water::Side& side,
                                   const Water::State& state) {
    if (!serial_port_->isOpen()) {
        logger_->Warn("[water]: Cannot force command; not connected");
        return;
    }

    // Force-disable auto compensation
    EnableAutoCompensation(false);

    // Modify or clear corresponding command bits
    switch (state) {
        case Water::State::kFilling:
            ModifyWaterCommand((side == Water::Side::kA) ? kAIn : kBIn);
            break;
        case Water::State::kDraining:
            ModifyWaterCommand((side == Water::Side::kA) ? kAOut : kBOut);
            break;
        case Water::State::kStopped:
        default:
            ClearWaterCommand(side);
    }

    logger_->Debug("[water]: Forced command to " + BytesToStr(water_cmd_)
                   + " (A_IN | B_IN | A_OUT | B_OUT)");
}
