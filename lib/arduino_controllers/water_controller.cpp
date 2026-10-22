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
#include <algorithm>
#include <bitset>
#include <cmath>

// Other Library Headers
//   (none)

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Arduino Commands (slots)
 * !Volume Tracking (slots)
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

// Volume Tracking

constexpr int kVolumeTrackPeriodMs = 1000 / 30;  // ms (30 Hz)

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Stringifies the lower 4 bits of a byte array for printing (Qt doesn't
 *        seem to support this natively).
 *
 * @param bytes Byte array to convert
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
    : GenericSerialDevice(parent, debug_mode, "Water"),
      volume_timer_(new QTimer(this)) {
    water_cmd_.resize(1);  // only 4 bits needed, so reserve 1 byte
    ClearCommand();

    // Track CW volumes for the app's whole lifetime (only changes when open)
    volume_timer_->setInterval(kVolumeTrackPeriodMs);
    connect(volume_timer_, &QTimer::timeout,  // integrate flow
            this, &WaterController::TrackVolume);
    volume_clock_.start();
    volume_timer_->start();
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Sends the current command to the WaterArduino.
 */
void WaterController::OnUpdate() {
    qint64 bytes_written = serial_port_->write(water_cmd_);
    if (bytes_written == -1) {
        throw std::runtime_error("Failed to write to port");
    }

    SendStatus(Water::Side::kA);  // emits ReportStatus
#if LIBRA_VERSION == 1
    SendStatus(Water::Side::kB);
#endif
}

/**
 * @brief Clears the current water command.
 *
 * @param side Fluid system side to clear
 */
void WaterController::ClearCommand(const Water::Side& side) {
    switch (side) {
        case Water::Side::kA:
            water_cmd_[0] &= static_cast<uint8_t>(~(kAIn | kAOut));
            break;
        case Water::Side::kB:
            water_cmd_[0] &= static_cast<uint8_t>(~(kBIn | kBOut));
            break;
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
 * @param new_bits Bit(s) for the new command(s)
 *
 * @see kAIn kBIn kAOut kBOut
 */
void WaterController::ModifyCommand(uint8_t new_bits) {
    new_bits &= kWaterCmdMask;  // ensure only lower 4 bits are used

    // Mutually exclusive: if a side's bit is modified, clear both bits first
    if ((new_bits & (kAIn | kAOut)) != 0) {
        ClearCommand(Water::Side::kA);
    }
    if ((new_bits & (kBIn | kBOut)) != 0) {
        ClearCommand(Water::Side::kB);
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
 * @see HebiThread::run
 */
void WaterController::MapTorqueToCommand(const double& torque_dir) {
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

    ModifyCommand(command);

    logger_->Debug("Set command to " + BytesToStr(water_cmd_)
                   + " (A_IN | B_IN | A_OUT | B_OUT)");
}

/**
 * @brief Parses the active water command based on the specified side and
 *        reports its status.
 *
 * @param side Fluid system side to check
 */
void WaterController::SendStatus(const Water::Side& side) {
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

    // Volume tracking follows valve state
    const auto index = static_cast<std::size_t>(side);
    if (index < Water::kSideCount) {
        tanks_[index].state = state;
    }

    emit ReportStatus(side, state);
}

/**
 * @brief Integrates each CW's flow rate into its volume and reports changes.
 *
 * @note Water only flows while connected, and never past empty or full.
 */
void WaterController::TrackVolume() {
    const double dt = volume_clock_.restart() / 1000.0;  // ms -> s

    for (std::size_t i = 0; i < Water::kSideCount; ++i) {
        Tank& tank = tanks_[i];
        const auto side = static_cast<Water::Side>(i);

        double flow = 0.0;
        if (IsOpen()) {
            if (tank.state == Water::State::kFilling
                && tank.volume < Water::kCapacity) {
                flow = tank.fill_rate;
            } else if (tank.state == Water::State::kDraining
                       && tank.volume > 0.0) {
                flow = -tank.drain_rate;
            }
        }
        if (flow == 0.0 && tank.flow == 0.0 && !tank.unreported) {
            continue;  // nothing new
        }

        const double old_volume = tank.volume;
        tank.volume = std::clamp(tank.volume + flow * dt, 0.0, Water::kCapacity);
        tank.flow = flow;
        tank.unreported = false;
        emit ReportVolume(side, tank.volume, tank.flow);

        // Only report limit reached once
        if (tank.volume != old_volume
            && (tank.volume == 0.0 || tank.volume == Water::kCapacity)) {
            emit TankLimitReached(side, tank.volume == Water::kCapacity);
        }
    }
}

//------------------------------------------------------------------------------
// !Arduino Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Sets the state of the fluid system's automatic torque compensation (ATC).
 *
 * @param enabled Whether to enable auto-compensation
 *
 * @see MapTorqueToCommand
 */
void WaterController::EnableAutoTorqueComp(const bool& enabled) {
    if (auto_comp_en_ == enabled) {
        return;  // no change, so do nothing
    }

    ClearCommand();  // reset the previous command when switching modes
    auto_comp_en_ = enabled;

    logger_->Info("Automatic torque compensation "
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
        MapTorqueToCommand(torque_dir.value());
    } else {
        ClearCommand();
    }
}

/**
 * @brief Forces the state of a specific side of the fluid system.
 *
 * @param side Side to command
 * @param state State to force
 */
void WaterController::ForceCommand(const Water::Side& side,
                                   const Water::State& state) {
    if (!serial_port_->isOpen()) {
        emit ErrorThrown("Water - Can't force command; not connected!");
        return;
    }

    // Force-disable automatic torque compensation
    EnableAutoTorqueComp(false);

    // Modify or clear corresponding command bits
    if (state == Water::State::kStopped) {
        ClearCommand(side);
    } else {
        uint8_t mask = 0;

        if (side == Water::Side::kA || side == Water::Side::kAll) {
            mask |= (state == Water::State::kFilling) ? kAIn : kAOut;
        }
        if (side == Water::Side::kB || side == Water::Side::kAll) {
            mask |= (state == Water::State::kFilling) ? kBIn : kBOut;
        }

        ModifyCommand(mask);
    }

    logger_->Info("Forced command to " + BytesToStr(water_cmd_)
                  + " (A_IN | B_IN | A_OUT | B_OUT)");
}

//------------------------------------------------------------------------------
// !Volume Tracking (slots)
//------------------------------------------------------------------------------

/**
 * @brief Updates a side's fill/drain rate from measured flow. In case of error,
 *        falls back to hard-coded rates.
 *
 * @param side Fluid system side
 * @param flow Net flow rate (mL/s); positive = inflow, negative = outflow,
 *             zero = idle, -1.0 = error
 */
void WaterController::UpdateFlowRate(const Water::Side& side,
                                     const double& flow) {
    const auto index = static_cast<std::size_t>(side);
    if (index >= Water::kSideCount) {
        logger_->Warn("Ignoring flow rate for unknown fluid system side: "
                      + Water::SideEnumToString(side));
        return;
    }

    Tank& tank = tanks_[index];
    tank.fill_rate = 0;   // default: Water::kFillRate;
    tank.drain_rate = 0;  // default: Water::kDrainRate;

    if (flow > 0.0) {
        tank.fill_rate = flow / 1000.0;  // mL/s -> L/s
    } else if (flow < 0.0 && flow != -1.0) {
        tank.drain_rate = -flow / 1000.0;  // mL/s -> L/s
    }
}

/**
 * @brief Overwrites the tracked volume (WARNING: see note).
 *
 * @param side Fluid system side
 * @param volume New volume (L); clamped to [0, capacity]
 *
 * @note ONLY TO BE USED IF INACCURACIES ARE NOTICED IN REAL TIME!!!
 */
void WaterController::OverrideVolume(const Water::Side& side,
                                     const double& volume) {
    if (!std::isfinite(volume)) {
        logger_->Warn("Ignoring invalid CW volume override");
        return;
    }

    for (std::size_t i = 0; i < Water::kSideCount; ++i) {
        if (side == Water::Side::kAll || static_cast<std::size_t>(side) == i) {
            tanks_[i].volume = std::clamp(volume, 0.0, Water::kCapacity);
            tanks_[i].unreported = true;
        }
    }

    logger_->Info("CW volume overridden to "
                  + QString::number(volume, 'f', 1).toStdString() + " L");
}
