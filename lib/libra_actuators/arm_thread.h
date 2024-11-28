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

/**
 * @brief Actuator control class for the LIBRA-II.
 *
 * @note This class is purposefully similar to `AbstractActuator`.
 *       It just provides a more intuitive interface for controlling the whole
 *       arm via one class, rather than by individually addressing each actuator.
 */
class ArmThread : QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    /**
     * @brief LIBRA joint names.
     */
    enum Joint { kYaw = 0, kPitch };

    ArmThread();
    ~ArmThread();

    // --- Arm Commands ---

    bool ConnectActuators();
    bool DisconnectActuators();

    void Move(double yaw, double pitch);
    void Stop();

    // --- Getters & Setters ---

    std::string GetStatus();

    std::vector<double> GetAllTargetPos();
    std::vector<double> GetAllActualPos();
    std::vector<double> GetAllActualTorque();

    double GetTargetPos(Joint joint);
    double GetActualPos(Joint joint);
    double GetActualTorque(Joint joint);

    void SetDebugMode(bool enabled);

  private:
    void run() override;

    // --- Helper Functions ---

    // --- Data Members ---

    bool debug_mode_{false};

    std::vector<AbstractActuator> actuators_;
};
