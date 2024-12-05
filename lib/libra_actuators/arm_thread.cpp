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
#include "epos_actuator.h"
#include "hebi_actuator.h"

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Arm Commands (Slots)
 */

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param actuators Vector of actuators to be constructed (see `actuator_defs.h`)
 * @param debug_mode Whether verbose debug text should be output
 */
ArmThread::ArmThread(QObject* parent, std::vector<ActuatorDef> actuator_defs,
                     bool debug_mode)
    : QThread(parent), debug_mode_(debug_mode) {
    // Initialize each actuator according to its type
    for (const auto& def : actuator_defs) {
        std::unique_ptr<AbstractActuator> actuator;

        switch (def.type) {
            case Actuator::Type::kEpos:
                {
                    // Access EPOS version of std::variant member
                    const auto epos_p = std::get<EposParams>(def.params);

                    actuator =
                        std::make_unique<EposActuator>(epos_p.device_name,
                                                       epos_p.protocol_name,
                                                       epos_p.interface_name,
                                                       epos_p.port_name,
                                                       epos_p.baud_rate,
                                                       debug_mode_);
                    break;
                }
            case Actuator::Type::kHebi:
                {
                    // Access HEBI version of std::variant member
                    const auto hebi_p = std::get<HebiParams>(def.params);

                    actuator = std::make_unique<HebiActuator>(hebi_p.families,
                                                              hebi_p.names,
                                                              debug_mode_);
                    break;
                }
            default:
                qCritical()
                    << "[ERROR] Cannot create actuator of undefined type!";
                continue;  // skip adding this entry to actuator array
        }

        // Save initialized actuator in joint order
        actuators_.at(def.joint) = std::move(actuator);
    }
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
    // Initialize these variables outside loop for efficiency
    std::vector<QString> statuses;

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // Send status updates from actuators
        for (const auto& actuator : actuators_) {
            statuses.push_back(QString::fromStdString(actuator->GetStatus()));
        }
        emit StatusChanged(statuses);
        statuses.clear();

        QThread::msleep(20);  // update 50 times/second
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {  // local to this file

}  // namespace

/**
 * @brief Returns the target position commanded to the actuator.
 *
 * @param joint Actuator to query
 * @return double Commanded position, in degrees
 */
double ArmThread::GetTargetPos(Actuator::Joint joint) {
    qDebug() << "TODO - ArmThread::GetTargetPos()";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the actual position of the actuator.
 *
 * @param joint Actuator to query
 * @return double Actual position, in degrees
 */
double ArmThread::GetActualPos(Actuator::Joint joint) {
    qDebug() << "TODO - ArmThread::GetActualPos()";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the actual effort (torque) of the actuator.
 *
 * @param joint Actuator to query
 * @return double Actual torque, in Newton-meters
 */
double ArmThread::GetActualTorque(Actuator::Joint joint) {
    qDebug() << "TODO - ArmThread::GetActualTorque()";
    return 0.0;

    // TODO: implementation
}

/**
 * @brief Returns the target positions commanded to all actuators.
 *
 * @return std::vector<double> Commanded positions, in degrees
 */
std::vector<double> ArmThread::GetAllTargetPos() {
    qDebug() << "TODO - ArmThread::GetAllTargetPos()";
    return {0.0};

    // TODO: implementation
}

/**
 * @brief Returns the current positions of all actuators.
 *
 * @return std::vector<double> Actual positions, in degrees
 */
std::vector<double> ArmThread::GetAllActualPos() {
    qDebug() << "TODO - ArmThread::GetAllActualPos()";
    return {0.0};

    // TODO: implementation
}

/**
 * @brief Returns the actual effort (torque) experienced by all actuators.
 *
 * @return std::vector<double> Actual torques, in Newton-meters
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
void ArmThread::SetDebugMode(const bool& enabled) {
    debug_mode_ = enabled;

    // Don't forget to update to all actuator objects
    for (const auto& actuator : actuators_) {
        actuator->SetDebugMode(enabled);
    }
}

/**
 * @brief Opens a connection with a specific actuator.
 *
 * @param joint Actuator to connect
 * @return true if successful, false otherwise
 */
void ArmThread::ConnectActuator(const Actuator::Joint& joint) {
    actuators_.at(joint)->Connect();
}

/**
 * @brief Closes the active connection to a specific actuator.
 *
 * @param joint Actuator to disconnect
 * @return true if successful, false otherwise
 */
void ArmThread::DisconnectActuator(const Actuator::Joint& joint) {
    actuators_.at(joint)->Disconnect();
}

/**
 * @brief Opens a connection with every actuator.
 */
void ArmThread::ConnectAllActuators() {
    for (const auto& actuator : actuators_) {
        actuator->Connect();
    }
}

/**
 * @brief Closes each active connection to an actuator.
 */
void ArmThread::DisconnectAllActuators() {
    for (const auto& actuator : actuators_) {
        actuator->Disconnect();
    }
}

/**
 * @brief Commands a specific actuator to start moving.
 *
 * @param joint Actuator to command
 * @param val Target value for actuator, in degrees
 */
void ArmThread::Move(const Actuator::Joint& joint, const double& val) {
    actuators_.at(joint)->Move(val);
}

/**
 * @brief Commands all connected actuators to start moving.
 *
 * @param vals Target values for all actuators (in `Joint` enum order), in degrees
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
