/******************************************************************************
 * @file   abstract_actuator.h
 * @brief  Abstract control class for LIBRA actuators; header file only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Other Library Headers
//   (none)

// Project Headers
//   (none)

/**
 * @brief Provides a common interface for a heterogeneous mix of actuators.
 *
 * @note If you are unfamiliar with abstract classes, they essentially just
 *       provide an interface for making multiple of types of similar objects.
 *       For example: `Fruit` can be used to define `Apple`, `Banana`, and
 *       `Orange`. Abstract classes contain common functions (defined normally)
 *       and pure virtual functions, denoted by the `= 0` at the end. This tells
 *       the compiler that it shouldn't allow a derivative class to compile if
 *       it doesn't first define those pure virtual functions (using `override`).
 */
class AbstractActuator {
  public:
    AbstractActuator() = default;
    AbstractActuator(bool debug_mode) : debug_mode_(debug_mode){};
    ~AbstractActuator() = default;

    // --- Actuator Commands ---

    virtual bool Connect() = 0;
    virtual bool Disconnect() = 0;

    virtual void Move(double deg) = 0;
    virtual void Stop() = 0;

    // --- Getters & Setters ---

    virtual std::string GetStatus() = 0;

    virtual double GetTargetPos() = 0;
    virtual double GetActualPos() = 0;
    virtual double GetActualTorque() = 0;

    void SetDebugMode(bool enabled) {
        debug_mode_ = enabled;
    };

  protected:
    // --- Helper Functions ---

    // --- Data Members ---

    bool debug_mode_{true};
};
