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

    void SetTarget(const std::vector<double>& deg) override;
    void Stop() override;

  signals:
    // --- Actuator Updates ---

    // NOTE: see `AbstractActuatorThread`

  private:
    void run() override;

    // --- Helper Functions ---

    // --- Data Members ---

    std::string device_name_;
    std::string protocol_name_;
    std::string interface_name_;
    std::string port_name_;
    uint baud_rate_;
};
