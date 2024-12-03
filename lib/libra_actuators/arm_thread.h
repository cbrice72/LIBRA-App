/******************************************************************************
 * @file   arm_thread.h
 * @brief  Control thread for LIBRA actuators; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <vector>

// Other Library Headers
#include <QThread>  // Qt::Core

// Project Headers
#include "abstract_actuator.h"
#include "actuator_defs.h"

/**
 * @brief Actuator control class for the LIBRA-II.
 *
 * @note This class is purposefully similar to `AbstractActuator`.
 *       It just provides a more intuitive interface for controlling the whole
 *       arm via one class, rather than by individually addressing each actuator.
 */
class ArmThread : public QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit ArmThread(QObject* parent, std::vector<ActuatorDef> actuators,
                       bool debug_mode_ = false);
    ~ArmThread() override = default;

  public slots:
    void SetDebugMode(bool enabled);

    // --- Arm Commands ---

    void ConnectActuator(const Joint& joint);
    void DisconnectActuator(const Joint& joint);

    void ConnectAllActuators();
    void DisconnectAllActuators();

    void Move(const Joint& joint, const double& val);
    void MoveAll(const std::vector<double>& vals);
    void Stop();

  signals:
    // --- Arm Updates ---

    void StatusChanged(const QString& status);
    void ErrorThrown(const QString& err);

  private:
    void run() override;

    // --- Helper Functions ---

    double GetTargetPos(Joint joint);
    double GetActualPos(Joint joint);
    double GetActualTorque(Joint joint);

    std::vector<double> GetAllTargetPos();
    std::vector<double> GetAllActualPos();
    std::vector<double> GetAllActualTorque();

    // --- Data Members ---

    bool debug_mode_;

    std::array<std::unique_ptr<AbstractActuator>, Joint::kJointCount> actuators_;
};
