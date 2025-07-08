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

// Project Headers
#include "util.h"
#ifdef BUILD_WITH_ROS2
# include "ros2_logger.h"
#else
# include "qt_logger.h"
#endif

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Thread Overrides
 * !Actuator Commands (slots)
 */

// Mechanical Properties

constexpr long kEncoderResolution = 500;  // Maxon part #: 228452
constexpr long kGearheadReduction = 113;  // Maxon part #: 203126

constexpr long kPinionTeeth = 15;    // KG Gear part #: SG1S15L-1010
constexpr long kSlewRingTeeth = 48;  // igus part #: PRT-04-50-TI-ST
constexpr double kSlewGearReduction = static_cast<double>(kSlewRingTeeth)
                                      / kPinionTeeth;

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

constexpr int kEposTrue = 1;       // TRUE
constexpr int kEposFalse = 0;      // FALSE
constexpr int32_t kTimeout = 100;  // ms
constexpr int kNodeID = 1;         // for now, we only support one actuator

// - For Profile Position/Velocity Modes

constexpr uint kProfileVel = 1000;  // rpm
constexpr uint kProfileAcc = 1000;  // rpm/s
constexpr uint kProfileDec = 100;   // rpm/s

constexpr uint kEmergencyProfileDec = 2000;  // rpm/s

// - For Profile Position Mode (see `VCS_MoveToPosition()`)

constexpr int kMoveAbsolute = 1;     // `Absolute` = TRUE
constexpr int kMoveRelative = 0;     // `Absolute` = FALSE
constexpr int kMoveImmediately = 1;  // `Immediately` = TRUE
constexpr int kMoveWaitForLast = 0;  // `Immediately` = FALSE

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param device_name Maxon device to connect to
 * @param protocol_name Communication protocol to use
 * @param interface_name Interface to communicate through
 * @param port_name Specific interface port where device is located
 * @param baud_rate Rate at which information will be transferred
 * @param debug_mode Whether to output verbose debug text
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
      baud_rate_(baud_rate),
      target_{0},
      last_target_{0}
#ifdef BUILD_WITH_ROS2
      ,
      rclcpp::Node("epos_node")
#endif
{
    // Initialize the logger
#ifdef BUILD_WITH_ROS2
    logger_ = std::make_unique<Ros2Logger>(debug_mode_, this->get_logger());
#else
    logger_ = std::make_unique<QtLogger>(debug_mode_);
#endif
}

/**
 * @brief Standard destructor.
 */
EposThread::~EposThread() {
    // This call to `Disconnect()` does three things:
    //   1) Ensures actuator comes to a complete stop
    //   2) Closes any and all EPOS devices on the network (i.e., all ports)
    //   3) Voids our class handle to the EPOS device
    Disconnect();

    logger_->Debug("Cleaned up EposThread");
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

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
    uint16_t state = 0;
    if (VCS_GetState(handle_, kNodeID, &state, &err_code) == 0) {
        emit ErrorThrown(
            "EPOS - Couldn't retrieve device state!\n"
            + util::GetFormattedEposErrTxt("VCS_GetState", err_code, kNodeID));
        return QStringLiteral("Error!");
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
    int a_vel = 0;  // rpm
    if (VCS_GetVelocityIsAveraged(handle_, kNodeID, &a_vel, &err_code) == 0) {
        emit ErrorThrown(
            "EPOS - Couldn't retrieve actual velocity!\n"
            + util::GetFormattedEposErrTxt("VCS_GetVelocityIsAveraged",
                                           err_code, kNodeID));
        return QStringLiteral("Error!");
    }

    int a_curr = 0;  // mA
    if (VCS_GetCurrentIsEx(handle_, kNodeID, &a_curr, &err_code) == 0) {
        emit ErrorThrown("EPOS - Couldn't retrieve current!\n"
                         + util::GetFormattedEposErrTxt("VCS_GetCurrentIsEx",
                                                        err_code, kNodeID));
        return QStringLiteral("Error!");
    }

    // Perform conversions
    const double vel = a_vel * kRpmToDegs;  // to deg/s
    const double curr = a_curr / 1000.0;    // to A

    // Create stringstream entry
    // - std::setw(7) for values to account for [sign][#,3][.][#,2]
    ss << "[" << kNodeID << "] - " << state_str << "\n"
       << "  Actual Velocity: " << std::setw(7) << vel << " deg/s\n"
       << "  Current:         " << std::setw(7) << curr << " A\n";

    return QString::fromStdString(ss.str());
}

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main command loop.
 */
void EposThread::run() {
    logger_->Debug("Initialized EposThread");

    // Initialize thread variables for efficiency
    uint err_code = 0;

    long t_pos = 0;
    int a_pos = 0;

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // To avoid Disconnect() voiding the handle_ in the middle of a loop
        // (which causes the app to crash), acquire the mutex lock on handle_
        std::unique_lock<std::mutex> lock(handle_mutex_);
        if (handle_ == nullptr) {
            emit ReportStatus(GetStatus(), type_);  // "Not Connected"
            lock.unlock();
            QThread::msleep(10);
            continue;
        }

        // Send movement command
        if (target_ != last_target_) {
            logger_->Debug("EPOS - Sending move command");

            // No complex trajectory-related logic necessary since we only
            // support ProfilePositionMode (for now)
            if (VCS_MoveToPosition(handle_, kNodeID, target_, kMoveAbsolute,
                                   kMoveImmediately, &err_code)
                == 0) {
                emit ErrorThrown(
                    "EPOS - Move command failed!\n"
                    + util::GetFormattedEposErrTxt("VCS_MoveToPosition",
                                                   err_code, kNodeID));
            }

            last_target_ = target_;  // mark the trajectory as "complete"
        }

        // Report important statuses individually
        if (VCS_GetTargetPosition(handle_, kNodeID, &t_pos, &err_code) == 0) {
            emit ErrorThrown(
                "EPOS - Failed to retrieve target position!\n"
                + util::GetFormattedEposErrTxt("VCS_GetTargetPosition",
                                               err_code, kNodeID));
        }
        // clang-format off
        emit ReportFeedback({{Actuator::Name::kYaw, static_cast<int32_t>(t_pos) * kIncToDeg}},
                            Actuator::Feedback::kTargetPos);
        // clang-format on

        if (VCS_GetPositionIs(handle_, kNodeID, &a_pos, &err_code) == 0) {
            emit ErrorThrown("EPOS - Failed to retrieve actual position!\n"
                             + util::GetFormattedEposErrTxt("VCS_GetPositionIs",
                                                            err_code, kNodeID));
        }
        // clang-format off
        emit ReportFeedback({{Actuator::Name::kYaw, a_pos * kIncToDeg}},
                            Actuator::Feedback::kActualPos);
        // clang-format on

        // Report minor statuses all together
        emit ReportStatus(GetStatus(), type_);

        // Release the mutex lock
        lock.unlock();

        QThread::msleep(10);  // update 100 times/second (theoretically)
    }
}

//------------------------------------------------------------------------------
// !Actuator Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish connections to all actuators.
 *
 * @return true Connection successful
 * @return false Connection failed
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
        emit ErrorThrown("EPOS - Couldn't open device!\n"
                         + util::GetFormattedEposErrTxt("VCS_OpenDevice",
                                                        err_code, kNodeID));
        return;
    }

    // Check if user-supplied baud rate is accepted by controller
    uint valid_rate = 0;
    std::stringstream valid_str;
    int end_of_sel = 0;
    bool baud_is_valid = false;
    if (VCS_GetBaudrateSelection(device_name_.data(), protocol_name_.data(),
                                 interface_name_.data(), port_name_.data(),
                                 kEposTrue, &valid_rate, &end_of_sel, &err_code)
        > 0) {
        // Skip the while loop below if we get it on the first try
        if (baud_rate_ == valid_rate) {
            baud_is_valid = true;
        }

        valid_str << std::to_string(valid_rate) << " ";

        // Else, keep getting more values
        while (!baud_is_valid && end_of_sel == 0) {
            // Query controller for next rate
            valid_rate = 0;
            VCS_GetBaudrateSelection(device_name_.data(), protocol_name_.data(),
                                     interface_name_.data(), port_name_.data(),
                                     kEposFalse, &valid_rate, &end_of_sel,
                                     &err_code);

            // In case this check fails, build list of valid rates to output
            valid_str << std::to_string(valid_rate) << " ";

            // Check last controller-provided rate
            if (baud_rate_ == valid_rate) {
                baud_is_valid = true;
            }
        }
    }

    if (!baud_is_valid) {
        emit ErrorThrown(QString::fromStdString(
            "EPOS - Unsupported baud rate; expecting: " + valid_str.str()));
        VCS_CloseDevice(handle, &err_code);
        return;
    }

    // Set controller baud rate and timeout
    if (VCS_SetProtocolStackSettings(handle, baud_rate_, kTimeout, &err_code)
        == 0) {
        emit ErrorThrown(
            "EPOS - Couldn't set baud rate!\n"
            + util::GetFormattedEposErrTxt("VCS_SetProtocolStackSettings",
                                           err_code, kNodeID));
        VCS_CloseDevice(handle, &err_code);
        return;
    }

    // Just in case, clear any faults persisting from previous operation
    if (VCS_ClearFault(handle, kNodeID, &err_code) == 0) {
        emit ErrorThrown("EPOS - Couldn't clear existing fault(s)!\n"
                         + util::GetFormattedEposErrTxt("VCS_ClearFault",
                                                        err_code, kNodeID));
        VCS_CloseDevice(handle, &err_code);
        return;
    }

    // Initialize our class target variables to the actuator's initial position
    int a_pos = 0;
    if (VCS_GetPositionIs(handle, kNodeID, &a_pos, &err_code) == 0) {
        emit ErrorThrown("EPOS - Failed to retrieve starting position!\n"
                         + util::GetFormattedEposErrTxt("VCS_GetPositionIs",
                                                        err_code, kNodeID));
        VCS_CloseDevice(handle, &err_code);
        return;
    }
    target_ = last_target_ = a_pos * kIncToDeg;

    // Initialize to Profile Position Mode by default
    if (VCS_SetOperationMode(handle, kNodeID, OMD_PROFILE_POSITION_MODE,
                             &err_code)
        == 0) {
        emit ErrorThrown("EPOS - Couldn't set operational mode to PPM!\n"
                         + util::GetFormattedEposErrTxt("VCS_SetOperationMode",
                                                        err_code, kNodeID));
        VCS_CloseDevice(handle, &err_code);
        return;
    }

    // Set position profile parameters
    if (VCS_SetPositionProfile(handle, kNodeID, kProfileVel, kProfileAcc,
                               kProfileDec, &err_code)
        == 0) {
        emit ErrorThrown(
            "EPOS - Couldn't set movement profile!\n"
            + util::GetFormattedEposErrTxt("VCS_SetPositionProfile", err_code,
                                           kNodeID));
        VCS_CloseDevice(handle, &err_code);
        return;
    }

    // Enable the controller
    if (VCS_SetEnableState(handle, kNodeID, &err_code) == 0) {
        emit ErrorThrown("EPOS - Couldn't enable controller!\n"
                         + util::GetFormattedEposErrTxt("VCS_SetEnableState",
                                                        err_code, kNodeID));
        VCS_CloseDevice(handle, &err_code);
        return;
    }

    handle_ = handle;  // only set our class handle after successful init
    emit Connected(true);

    logger_->Debug("EPOS - Connection successful");
}

/**
 * @brief Terminates the active connection(s).
 */
void EposThread::Disconnect() {
    if (handle_ == nullptr) {
        return;  // do nothing
    }

    // If this was called in the middle of a movement, gracefully stop
    Stop();

    // Acquire a mutex lock on handle before doing sensitive operations
    // (NOTE: automatically unlocked when function ends)
    std::lock_guard<std::mutex> lock(handle_mutex_);

    // Close the connection via the EPOS API
    uint err_code = 0;
    if (VCS_CloseDevice(handle_, &err_code) == 0) {
        emit ErrorThrown("EPOS - Problem closing device!\n"
                         + util::GetFormattedEposErrTxt("VCS_CloseDevice",
                                                        err_code, kNodeID));
        return;
    }

    // Void our class handle
    handle_ = nullptr;
    emit Connected(false);

    logger_->Debug("EPOS - Gracefully disconnected from actuator");
}

/**
 * @brief Sets movement target/trajectory for the connected actuator.
 *
 * @param target Target angle (absolute), in degrees
 *
 * @note I don't see a reason to use `EposThread` outside of the LIBRA project,
 *       at least in the near future. Since LIBRA-II only uses a single EPOS
 *       actuator, I won't go through the trouble of implementing the ability to
 *       command any N actuators, like in `HebiThread::SetTarget()`.
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

    logger_->Debug("EPOS - Set target(s) to " + std::to_string(target_)
                   + " inc");
}

/**
 * @brief Halts the trajectory of the actuator.
 */
void EposThread::Stop() {
    if (handle_ == nullptr) {
        return;  // do nothing
    }

    // Reset class target variables to current position
    uint err_code = 0;
    if (VCS_GetPositionIs(handle_, kNodeID, &target_, &err_code) == 0) {
        emit ErrorThrown("EPOS - Failed to retrieve actual position!\n"
                         + util::GetFormattedEposErrTxt("VCS_GetPositionIs",
                                                        err_code, kNodeID));
        return;
    }
    last_target_ = target_;  // disables MoveToPosition block in run()

    // Set emergency stop profile deceleration
    if (VCS_SetPositionProfile(handle_, kNodeID, kProfileVel, kProfileAcc,
                               kEmergencyProfileDec, &err_code)
        == 0) {
        emit ErrorThrown(
            "EPOS - Couldn't set emergency deceleration profile!\n"
            + util::GetFormattedEposErrTxt("VCS_SetPositionProfile", err_code,
                                           kNodeID));
        return;
    }

    // Stop the actuator via the EPOS API
    if (VCS_HaltPositionMovement(handle_, kNodeID, &err_code) == 0) {
        emit ErrorThrown(
            "EPOS - Couldn't stop actuator!\n"
            + util::GetFormattedEposErrTxt("VCS_HaltPositionMovement", err_code,
                                           kNodeID));
        return;
    }

    // Reset profile deceleration to original value
    if (VCS_SetPositionProfile(handle_, kNodeID, kProfileVel, kProfileAcc,
                               kProfileDec, &err_code)
        == 0) {
        emit ErrorThrown(
            "EPOS - Couldn't restore movement profile!\n"
            + util::GetFormattedEposErrTxt("VCS_SetPositionProfile", err_code,
                                           kNodeID));
        return;
    }

    logger_->Debug("EPOS - Actuator stopped");
}
