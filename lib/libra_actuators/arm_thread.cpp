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
#include <QDebug>  // Qt::Core

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Arm Commands (Slots)
 */

/**
 * @brief Standard constructor.
 *
 * @todo --------------------------------------
 *       THIS IS WHERE I LEFT OFF ON 2024/11/29
 *       --------------------------------------
 *       Add parameter(s) to the constructor. We need to be able to infer the 1)
 *       number of actuators and 2) which actuator class to make instances of.
 *       This might mean moving the `Joint` enum out of `ArmThread` and defining
 *       it at the application level (i.e., in the `MainWindow` class).
 */
ArmThread::ArmThread(QObject* parent, bool debug_mode)
    : QThread(parent), debug_mode_(debug_mode) {
    qDebug() << "TODO - ArmThread::ArmThread()";

    // TODO: implementation
}

/**
 * @brief Standard destructor.
 *
 * @note If QThread's destructor needs to be overridden, clear the `= default`
 *       in this destructor's declaration (see `arm_thread.h`).
 */
/*
ArmThread::~ArmThread() {
    qDebug() << "TODO";

    // TODO: implementation
}
*/

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

/**
 * @brief TODO: description.
 *
 * @param joint
 * @return double
 */
double ArmThread::GetTargetPos(Joint joint) {
    qDebug() << "TODO - ArmThread::GetTargetPos()";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @param joint
 * @return double
 */
double ArmThread::GetActualPos(Joint joint) {
    qDebug() << "TODO - ArmThread::GetActualPos()";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @param joint
 * @return double
 */
double ArmThread::GetActualTorque(Joint joint) {
    qDebug() << "TODO - ArmThread::GetActualTorque()";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @return std::vector<double>
 */
std::vector<double> ArmThread::GetAllTargetPos() {
    qDebug() << "TODO - ArmThread::GetAllTargetPos()";
    return {0.0};

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @return std::vector<double>
 */
std::vector<double> ArmThread::GetAllActualPos() {
    qDebug() << "TODO - ArmThread::GetAllActualPos()";
    return {0.0};

    // TODO: implementation
}

/**
 * @brief TODO: description.
 *
 * @return std::vector<double>
 */
std::vector<double> ArmThread::GetAllActualTorque() {
    qDebug() << "TODO - ArmThread::GetAllActualTorque()";
    return {0.0};

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Arm Commands (Slots)
//------------------------------------------------------------------------------

/**
 * @brief Controls the output of verbose debug text
 * @param true to enable, false to disable
 */
void ArmThread::SetDebugMode(bool enabled) {
    debug_mode_ = enabled;
}

/**
 * @brief Opens a connection with a specific actuator.
 *
 * @return true if successful, false otherwise
 */
void ArmThread::ConnectActuator(const Joint& joint) {
    qDebug() << "TODO - ArmThread::ConnectActuator()";

    // TODO: implementation
}

/**
 * @brief Closes the active connection to a specific actuator.
 *
 * @return true if successful, false otherwise
 */
void ArmThread::DisconnectActuator(const Joint& joint) {
    qDebug() << "TODO - ArmThread::DisconnectActuator()";

    // TODO: implementation
}

/**
 * @brief Opens a connection with every actuator.
 *
 * @return true if successful (for all actuators), false otherwise
 */
void ArmThread::ConnectAllActuators() {
    qDebug() << "TODO - ArmThread::ConnectAllActuators()";

    // TODO: implementation
}

/**
 * @brief Closes each active connection to an actuator.
 *
 * @return true if successful (for all actuators), false otherwise
 */
void ArmThread::DisconnectAllActuators() {
    qDebug() << "TODO - ArmThread::DisconnectAllActuators()";

    // TODO: implementation
}

/**
 * @brief Commands a specific actuator to start moving.
 *
 * @param joint Actuator to command
 * @param val Target value for actuator, in degrees
 */
void ArmThread::Move(const Joint& joint, const double& val) {
    actuators_.at(joint)->Move(val);
}

/**
 * @brief Commands all connected actuators to start moving.
 *
 * @param vals Target value for all actuators (in `Joint` enum order), in degrees
 */
void ArmThread::MoveAll(const std::vector<double>& vals) {
    if (actuators_.size() != vals.size()) {
        emit ErrorThrown(QString("Number of values provided (%1) does not "
                                 "match number of actuators (%2)")
                             .arg(vals.size(), actuators_.size()));
        return;
    }

    for (auto i = 0; i < actuators_.size(); i++) {
        actuators_.at(i)->Move(vals.at(i));
    }
}

/**
 * @brief Commands all connected actuators to stop moving.
 */
void ArmThread::Stop() {
    for (const auto& actuator : actuators_) {
        actuator->Stop();
    }
}
