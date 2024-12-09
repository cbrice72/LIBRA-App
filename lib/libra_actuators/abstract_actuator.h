/******************************************************************************
 * @file   abstract_actuator.h
 * @brief  Abstract control class for LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Other Library Headers
#include <QThread>  // Qt::Core

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
class AbstractActuatorThread : public QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    AbstractActuatorThread() = default;
    AbstractActuatorThread(QObject* parent, bool debug_mode)
        : QThread(parent), debug_mode_(debug_mode){};
    ~AbstractActuatorThread() override = default;

  public slots:

    void SetDebugMode(const bool& enabled) {
        debug_mode_ = enabled;
    };

    // --- Actuator Commands ---

    virtual void Connect() = 0;
    virtual void Disconnect() = 0;

    virtual void SetTarget(const std::vector<double>& deg) = 0;
    virtual void Stop() = 0;

  signals:
    // --- Actuator Updates ---

    void ReportTargetPos(const std::vector<double>& target_position);
    void ReportActualPos(const std::vector<double>& actual_position);
    void ReportActualTorque(const std::vector<double>& actual_torque);

    void ReportStatus(const std::vector<QString>& statuses);
    void ErrorThrown(const QString& err);

  protected:
    // --- Helper Functions ---

    // --- Data Members ---

    bool debug_mode_{true};
};
