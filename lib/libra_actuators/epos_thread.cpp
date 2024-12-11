/******************************************************************************
 * @file   epos_thread.cpp
 * @brief  Control class for EPOS (Maxon) actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "epos_thread.h"

// C++ Standard Library Headers
//   (none)

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

// NOTE: in the following conversion, encoder resolution is multiplied by 4.0
//       because the EPOS4 Firmware Specification says so (see "Digital
//       incremental encoder" section, p. 156)
constexpr double kDegToInc = (kEncoderResolution * 4.0 * kGearheadReduction
                              * kSlewGearReduction)
                             / 360;
constexpr double kIncToDeg = 360
                             / (kEncoderResolution * 4.0 * kGearheadReduction
                                * kSlewGearReduction);

// EPOS Functions

constexpr int32_t kTimeout = 3000;  // ms

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
    // This call to `Disconnect()` does two things:
    //   1) Ensures actuators come to a complete stop
    //   2) Ensures the EPOS handle gets cleaned up
    Disconnect();
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {  // local to this file

}  // namespace

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
    //   (none)

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // TODO: implementation

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
    handle_ = VCS_OpenDevice(device_name_.data(), protocol_name_.data(),
                             interface_name_.data(), port_name_.data(),
                             &err_code);

    if (handle_ == nullptr || err_code != 0) {
        // emit ErrorThrown(util::GetEPOSErr("VCS_OpenDevice", err_code));
        qCritical() << "[TEMPORARY]\nEPOS - Couldn't open device!";
        return;
    }

    // Set controller baud rate and timeout
    if (VCS_SetProtocolStackSettings(handle_, baud_rate_, kTimeout, &err_code)
        <= 0) {
        // emit ErrorThrown(util::GetEPOSErr("VCS_SetProtocolStackSettings", err_code));
        qCritical() << "[TEMPORARY]\nEPOS - Couldn't set baud rate!";
        VCS_CloseDevice(handle_, &err_code);
        return;
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
            // emit ErrorThrown(util::GetEPOSErr("VCS_CloseDevice", err_code));
            qCritical() << "[TEMPORARY]\nEPOS - Problem closing device!";
            return;
        }

        // Void our local handle
        handle_ = nullptr;
    }
}

/**
 * @brief Sends movement commands to all connected actuators.
 *
 * @param deg Target angles (absolute) for all actuators, in degrees
 */
void EposThread::SetTarget(const std::vector<double>& deg) {
    qWarning() << "TODO - EposThread::Move()";

    // TODO: implementation
}

/**
 * @brief Halts the trajectories of all actuators.
 */
void EposThread::Stop() {
    qWarning() << "TODO - EposThread::Stop()";

    // TODO: implementation
}
