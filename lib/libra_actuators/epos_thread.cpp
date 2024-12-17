/******************************************************************************
 * @file   epos_thread.cpp
 * @brief  Control class for EPOS (Maxon) actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "epos_thread.h"

// C++ Standard Library Headers
#include <iomanip>
#include <sstream>

// Other Library Headers
#include "Definitions.h"  // EPOS (Maxon)
#include <QDebug>         // Qt::Core

// Project Headers
// #include "util.h"  // TODO: integrate with main project

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Thread Overrides
 * !Actuator Commands (slots)
 */

/* Constants */

// Mechanical Properties

constexpr long kEncoderResolution = 500;  // Maxon part #: 228452
constexpr long kGearheadReduction = 113;  // Maxon part #: 203126

constexpr long kPinionTeeth = 15;    // KG Gear part #: SG1S15L-1010
constexpr long kSlewRingTeeth = 48;  // igus part #: PRT-04-50-TI-ST
/*
constexpr double kSlewGearReduction = static_cast<double>(kSlewRingTeeth)
                                      / kPinionTeeth;
*/
constexpr double kSlewGearReduction = 1.0;  // FOR TESTING PURPOSES ONLY

// Improving Readability of Conversions

constexpr long kDegPerRotation = 360;
constexpr long kSecPerMin = 60;

// NOTE: in the following conversions, encoder resolution is multiplied by 4
//       because it is a quadrature encoder (see EPOS4 Firmware Specification
//       "Digital incremental encoder" section, p. 156)
constexpr double kDegToInc = (kEncoderResolution * 4.0 * kGearheadReduction
                              * kSlewGearReduction)
                             / kDegPerRotation;
constexpr double kIncToDeg = kDegPerRotation
                             / (kEncoderResolution * 4.0 * kGearheadReduction
                                * kSlewGearReduction);

constexpr double kDegsToRpm = (kSecPerMin * kGearheadReduction
                               * kSlewGearReduction)
                              / kDegPerRotation;
constexpr double kRpmToDegs = kDegPerRotation
                              / (kSecPerMin * kGearheadReduction
                                 * kSlewGearReduction);

// EPOS Functions

constexpr int32_t kTimeout = 3000;  // ms
constexpr int kNodeID = 1;          // for now, we only support one actuator

// - For Profile Position/Velocity Modes
constexpr uint kProfileVel = 1000;  // rpm
constexpr uint kProfileAcc = 1000;  // rpm/s
constexpr uint kProfileDec = 100;   // rpm/s

// - For Profile Position Mode (see `VCS_MoveToPosition()`)
constexpr int kMoveAbsolute = 1;     // `Absolute` = TRUE
constexpr int kMoveRelative = 0;     // `Absolute` = FALSE
constexpr int kMoveImmediately = 1;  // `Immediately` = TRUE
constexpr int kMoveWaitForLast = 0;  // `Immediately` = FALSE

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param device_name Maxon device to connect to
 * @param protocol_name Communication protocol to use
 * @param interface_name Interface to communicate through
 * @param port_name Specific interface port where device is located
 * @param baud_rate Rate at which information will be transferred
 * @param debug_mode Whether verbose debug text should be output
 */
EposThread::EposThread(QObject* parent, std::string device_name,
                       std::string protocol_name, std::string interface_name,
                       std::string port_name, uint baud_rate,
                       const bool& debug_mode)
    : AbstractActuatorThread(parent, debug_mode, Actuator::Type::kEpos),
      device_name_(std::move(device_name)),
      protocol_name_(std::move(protocol_name)),
      interface_name_(std::move(interface_name)),
      port_name_(std::move(port_name)),
      baud_rate_(baud_rate) {}

/**
 * @brief Standard destructor.
 *
 */
EposThread::~EposThread() {
    // Since the EPOS library provides two "CloseDevice"-type functions, we use
    // the more general one here to be safe (instead of calling our `Disconnect()`)
    uint err_code = 0;
    if (VCS_CloseAllDevices(&err_code) == 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS - VCS_CloseAllDevices", err_code));
        emit ErrorThrown("[TEMPORARY]\nEPOS - Failed to close all devices!");
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {  // local to this file

}  // namespace

/**
 * @brief Returns minor status information for the connected actuator.
 *        Target position, actual position, and actual torque (effort) are
 *        provided separately as signals.
 *
 * @return QString Comma-delimited status messages
 *
 * @note See following HEBI C++ API page for full list of available
 * feedback:
 *       https://files.hebi.us/docs/cpp/cpp-3.11.1/classhebi_1_1GroupFeedback.html
 */
QString EposThread::GetStatus() {
    if (handle_ == nullptr) {
        return {QString("Not Connected")};
    }

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);  // 0.01
    ss << std::right;                          // right-align numbers

    // Get device state and convert to string
    uint err_code = 0;
    unsigned short int state = 0;
    if (VCS_GetState(handle_, kNodeID, &state, &err_code) == 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS - VCS_GetState", err_code,
        // kNodeID));
        emit ErrorThrown("[TEMPORARY]\nEPOS - Couldn't retrieve device state!");
    }

    std::string state_str;
    if (state == 0) {  // ST_DISABLED
        state_str = "disabled";
    } else if (state == 1) {  // ST_ENABLED
        state_str = "enabled";
    } else if (state == 2) {  // ST_QUICKSTOP
        state_str = "quickstop";
    } else if (state == 3) {  // ST_FAULT
        state_str = "fault";
    }

    // Get values
    int a_vel = 0;
    if (VCS_GetVelocityIsAveraged(handle_, kNodeID, &a_vel, &err_code) <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS -
        // VCS_GetVelocityIsAveraged", err_code, kNodeID));
        emit ErrorThrown(
            "[TEMPORARY]\nEPOS - Couldn't retrieve actual velocity!");
    }
    a_vel *= kRpmToDegs;

    int curr = 0;
    if (VCS_GetCurrentIsEx(handle_, kNodeID, &curr, &err_code) <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS -
        // VCS_GetCurrentIsEx", err_code, kNodeID));
        emit ErrorThrown("[TEMPORARY]\nEPOS - Couldn't retrieve current!");
    }

    // Create stringstream entry
    // - std::setw(7) for values to account for [sign][#,3][.][#,2]
    ss << "[" << kNodeID << "] - " << state_str << "\n"
       << "  Actual Velocity: " << std::setw(7) << a_vel << " deg/s\n"
       << "  Current:         " << std::setw(7) << curr << " A\n";

    return QString::fromStdString(ss.str());
}

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main pump command loop.
 */
void EposThread::run() {
    if (debug_mode_) {
        qDebug() << "[DEBUG] Initialized EposThread";
    }

    // Initialize thread variables for efficiency
    uint err_code = 0;

    long t_pos = 0;
    int a_pos = 0;

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        if (handle_ != nullptr) {
            // Send movement command
            if (target_ != last_target_) {
                if (debug_mode_) {
                    qDebug() << "EPOS - Sending move command";
                }

                // No complex trajectory-related logic necessary since we only
                // support ProfilePositionMode (for now)
                if (VCS_MoveToPosition(handle_, kNodeID, target_, kMoveAbsolute,
                                       kMoveImmediately, &err_code)
                    <= 0) {
                    // emit ErrorThrown(util::PrintEPOSErr("EPOS -
                    // VCS_MoveToPosition", err_code, kNodeID));
                    emit ErrorThrown(
                        "[TEMPORARY]\nEPOS - Move command failed!");
                }

                last_target_ = target_;  // mark the trajectory as "complete"
            }

            // Report important statuses individually
            if (VCS_GetTargetPosition(handle_, kNodeID, &t_pos, &err_code)
                <= 0) {
                // emit ErrorThrown(util::PrintEPOSErr("EPOS -
                // VCS_GetTargetPosition", err_code, kNodeID));
                emit ErrorThrown(
                    "[TEMPORARY]\nEPOS - Failed to retrieve target position!");
            }
            // clang-format off
            emit ReportFeedback({{Actuator::Joint::kYaw, t_pos * kIncToDeg}},
                                Actuator::Feedback::kTargetPos);
            // clang-format on

            if (VCS_GetPositionIs(handle_, kNodeID, &a_pos, &err_code) <= 0) {
                // emit ErrorThrown(util::PrintEPOSErr("EPOS -
                // VCS_GetPositionIs", err_code, kNodeID));
                emit ErrorThrown(
                    "[TEMPORARY]\nEPOS - Failed to retrieve actual position!");
            }
            // clang-format off
            emit ReportFeedback({{Actuator::Joint::kYaw, a_pos * kIncToDeg}},
                                Actuator::Feedback::kActualPos);
            // clang-format on
        }

        // Report minor statuses all together
        emit ReportStatus(GetStatus(), type_);

        QThread::msleep(10);  // update 100 times/second (theoretically)
    }
}

//------------------------------------------------------------------------------
// !Actuator Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish connections to all actuators.
 *
 * @return true if successful, false otherwise
 */
void EposThread::Connect() {
    // If there is already an active connection, gracefully terminate it
    Stop();
    Disconnect();

    // Connect to specified controller
    uint err_code = 0;
    auto* handle = VCS_OpenDevice(device_name_.data(), protocol_name_.data(),
                                  interface_name_.data(), port_name_.data(),
                                  &err_code);

    if (handle == nullptr || err_code != 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS - VCS_OpenDevice", err_code));
        emit ErrorThrown("[TEMPORARY]\nEPOS - Couldn't open device!");
        return;
    }

    // Set controller baud rate and timeout
    if (VCS_SetProtocolStackSettings(handle, baud_rate_, kTimeout, &err_code)
        <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS -
        // VCS_SetProtocolStackSettings", err_code));
        emit ErrorThrown("[TEMPORARY]\nEPOS - Couldn't set baud rate!");
        VCS_CloseDevice(handle, &err_code);
        return;
    }

    // Just in case, clear any faults persisting from previous operation
    if (VCS_ClearFault(handle, kNodeID, &err_code) <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS - VCS_ClearFault",
        // err_code, kNodeID));
        emit ErrorThrown(
            "[TEMPORARY]\nEPOS - Couldn't clear existing fault(s)!");
        return;
    }

    // Initialize to Profile Position Mode by default
    if (VCS_SetOperationMode(handle, kNodeID, OMD_PROFILE_POSITION_MODE,
                             &err_code)
        <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS - VCS_SetOperationMode",
        // err_code, kNodeID));
        emit ErrorThrown(
            "[TEMPORARY]\nEPOS - Couldn't set operational mode to PPM!");
        return;
    }

    // Set position profile parameters
    if (VCS_SetPositionProfile(handle, kNodeID, kProfileVel, kProfileAcc,
                               kProfileDec, &err_code)
        <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS - VCS_SetPositionProfile",
        // err_code, kNodeID));
        emit ErrorThrown("[TEMPORARY]\nEPOS - Couldn't set movement profile!");
        return;
    }

    handle_ = handle;  // only set our class handle after successful init

    if (debug_mode_) {
        qDebug() << "EPOS - Connection successful";
    }
}

/**
 * @brief Terminates the active connection(s).
 *
 * @return true if successful, false otherwise
 */
void EposThread::Disconnect() {
    // In case this was called in the middle of a movement, gracefully stop
    Stop();

    if (handle_ != nullptr) {
        // Close the connection via the EPOS API
        uint err_code = 0;
        VCS_CloseDevice(handle_, &err_code);
        if (err_code != 0) {
            // emit ErrorThrown(util::PrintEPOSErr("EPOS - VCS_CloseDevice", err_code));
            emit ErrorThrown("[TEMPORARY]\nEPOS - Problem closing device!");
            return;
        }

        // Void our class handle
        handle_ = nullptr;
    }

    if (debug_mode_) {
        qDebug() << "EPOS - Gracefully disconnected from actuator";
    }
}

/**
 * @brief Sets movement target/trajectory for the connected actuator.
 *
 * @param target Target angle (absolute), in degrees
 *
 * @note Since I don't see a reason to use `EposThread` outside of the LIBRA
 *       project in the near future, and I only need at most one EPOS actuator,
 *       I won't go through the trouble of making full use of the EPOS library
 *       to match the HEBI API's one-group-to-many-actuators functionality.
 */
void EposThread::SetTarget(const std::vector<double>& target) {
    if (handle_ == nullptr) {
        return;  // do nothing
    }

    // Validate input
    if (target.size() != 1) {
        emit ErrorThrown("EPOS - Size of command vector != number of "
                         "supported actuators (1)!");
        return;
    }

    // Convert and save
    target_ = target.at(0) * kDegToInc;

    if (debug_mode_) {
        qDebug() << "EPOS - Set target to" << target_ << "inc";
    }
}

/**
 * @brief Halts the trajectory of the actuator.
 */
void EposThread::Stop() {
    if (handle_ == nullptr) {
        return;  // do nothing
    }

    // Stop the actuator
    uint err_code = 0;
    if (VCS_HaltPositionMovement(handle_, kNodeID, &err_code) <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS -
        // VCS_HaltPositionMovement", err_code, kNodeID));
        emit ErrorThrown("[TEMPORARY]\nEPOS - Couldn't stop actuator!");
    }

    // Reset class target variables to current position
    int current_pos = 0;
    if (VCS_GetPositionIs(handle_, kNodeID, &current_pos, &err_code) <= 0) {
        // emit ErrorThrown(util::PrintEPOSErr("EPOS -
        // VCS_GetPositionIs", err_code, kNodeID));
        emit ErrorThrown(
            "[TEMPORARY]\nEPOS - Failed to retrieve actual position!");
    }
    target_ = last_target_ = current_pos;

    if (debug_mode_) {
        qDebug() << "EPOS - Actuator stopped";
    }
}
