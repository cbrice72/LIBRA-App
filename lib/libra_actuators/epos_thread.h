/******************************************************************************
 * @file   epos_thread.h
 * @brief  Control class for LIBRA EPOS (Maxon) actuators; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Other Library Headers
//   (none)

// Project Headers
#include "abstract_actuator.h"

/**
 * @brief Control class for EPOS (Maxon) actuators.
 *
 * @note Since I don't see a reason to use `EposThread` outside of the LIBRA
 *       project in the near future, and I only need at most one EPOS actuator,
 *       I won't go through the trouble of making full use of the EPOS library
 *       to match the HEBI API's one-group-to-many-actuators functionality.
 *
 * @see abstract_actuator.h
 */
class EposThread : public AbstractActuatorThread {
  public:
    explicit EposThread(QObject* parent, std::string device_name,
                        std::string protocol_name, std::string interface_name,
                        std::string port_name, uint baud_rate,
                        const bool& debug_mode = false);
    ~EposThread() override;

  public slots:
    // --- Actuator Commands ---

    void Connect() override;
    void Disconnect() override;

    void SetTarget(const std::vector<double>& target) override;
    void Stop() override;

  signals:
    // --- Actuator Updates ---

    // NOTE: see `AbstractActuatorThread`

  private:
    void run() override;

    // --- Helper Functions ---

    QString GetStatus();

    // --- Data Members ---

    std::string device_name_;
    std::string protocol_name_;
    std::string interface_name_;
    std::string port_name_;
    uint baud_rate_;

    void* handle_{nullptr};  // void* are dangerous, but Maxon handles use them

    int target_{0};  // in inc, not deg
    int last_target_{0};
};
