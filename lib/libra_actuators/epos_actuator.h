/******************************************************************************
 * @file   epos_actuator.h
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
 *        Derived from `AbstractActuator` to provide a common interface.
 *
 * @note See `abstract_actuator.h`.
 */
class EposActuator : public AbstractActuator {
  public:
    explicit EposActuator(bool debug_mode = false);
    ~EposActuator();

    // --- Actuator Commands ---

    bool Connect() override;
    bool Disconnect() override;

    void Move(double deg) override;
    void Stop() override;

    // --- Getters & Setters ---

    std::string GetStatus() override;

    double GetTargetPos() override;
    double GetActualPos() override;
    double GetActualTorque() override;

  private:
    // --- Helper Functions ---

    // --- Data Members ---
};
