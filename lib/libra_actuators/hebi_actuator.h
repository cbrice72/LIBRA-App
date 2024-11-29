/******************************************************************************
 * @file   hebi_actuator.h
 * @brief  Control class for LIBRA HEBI actuators; header file.
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
 * @brief Control class for HEBI actuators.
 *        Derived from `AbstractActuator` to provide a common interface.
 *
 * @note See `abstract_actuator.h`.
 */
class HebiActuator : public AbstractActuator {
  public:
    explicit HebiActuator(bool debug_mode = false);
    ~HebiActuator();

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
