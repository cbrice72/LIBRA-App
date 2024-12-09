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
#include <QDebug>  // Qt::Core

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Thread Overrides
 * !Actuator Commands (slots)
 */

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
    : AbstractActuatorThread(parent, debug_mode),
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
    qWarning() << "TODO - EposThread::~EposThread()";

    // TODO: implementation
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

        QThread::msleep(100);  // update 10 times/second
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
    qWarning() << "TODO - EposThread::Connect()";

    // TODO: implementation
}

/**
 * @brief Terminates the active connection(s).
 *
 * @return true if successful, false otherwise
 */
void EposThread::Disconnect() {
    qWarning() << "TODO - EposThread::Disconnect()";

    // TODO: implementation
}

/**
 * @brief Sends movement commands to all connected actuators.
 *
 * @param deg Target angles (absolute) for all actuators
 *
 * @note If an actuator doesn't directly accept degrees as part of its
 *       movement command (i.e., if two actuators work together to effect two
 *       axes), the translation should be done before calling this function.
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
