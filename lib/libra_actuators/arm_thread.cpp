/******************************************************************************
 * @file   arm_thread.cpp
 * @brief  Control thread for LIBRA actuators; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "arm_thread.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
//   (none)

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Arm Commands
 * !Getters & Setters
 */

/**
 * @brief Standard constructor.
 */
ArmThread::ArmThread() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief Standard destructor.
 */
ArmThread::~ArmThread() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief TODO: description.
 */
void ArmThread::run() {
    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // TODO: implementation

        QThread::msleep(20);  // update 50 times/second
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Arm Commands
//------------------------------------------------------------------------------

/**
 * @brief Opens a connection with every actuator.
 *
 * @return true if successful (for all actuators), false otherwise
 */
bool ArmThread::ConnectActuators() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief Closes each active connection to an actuator.
 *
 * @return true if successful (for all actuators), false otherwise
 */
bool ArmThread::DisconnectActuators() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief Commands all connected actuators to start moving.
 *
 * @param yaw Target value for yaw joint, in degrees
 * @param pitch Target value for pitch joint, in degrees
 */
void ArmThread::Move(double yaw, double pitch) {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief Commands all connected actuators to stop moving.
 */
void ArmThread::Stop() {
    qDebug() << "TODO";

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief TODO: description.
 *
 * @return std::string
 */
std::string ArmThread::GetStatus() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @return std::vector<double>
 */
std::vector<double> ArmThread::GetAllTargetPos() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @return std::vector<double>
 */
std::vector<double> ArmThread::GetAllActualPos() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @return std::vector<double>
 */
std::vector<double> ArmThread::GetAllActualTorque() {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @param joint
 * @return double
 */
double ArmThread::GetTargetPos(Joint joint) {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @param joint
 * @return double
 */
double ArmThread::GetActualPos(Joint joint) {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @param joint
 * @return double
 */
double ArmThread::GetActualTorque(Joint joint) {
    qDebug() << "TODO";

    // TODO: implementation
}

/**
 * @brief Controls the output of verbose debug text
 * @param true to enable, false to disable
 */
void ArmThread::SetDebugMode(bool enabled) {
    debug_mode_ = enabled;
}
