/******************************************************************************
 * @file   abstract_actuator_thread.h
 * @brief  Abstract control class for LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
#include <unordered_map>

// Other Library Headers
#include <QThread>  // Qt::Core

// Project Headers
#include "actuator_defs.h"
#include "joint_defs.h"
#include "logger.h"

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
    AbstractActuatorThread(QObject* parent, bool debug_mode, Actuator::Type type)
        : QThread(parent), debug_mode_(debug_mode), type_(type){};
    ~AbstractActuatorThread() override = default;

  public slots:

    void SetDebugMode(const bool& enabled) {
        debug_mode_ = enabled;
        logger_->SetDebugMode(debug_mode_);
    };

    // --- Actuator Commands ---

    virtual void Connect() = 0;
    virtual void Disconnect() = 0;

    virtual void SetTarget(const std::vector<double>& deg) = 0;
    virtual void Stop() = 0;

  signals:
    // --- Actuator Updates ---

    void Connected(const bool& connected);

    void ReportFeedback(const std::unordered_map<Joint::Name, double>& feedbacks,
                        const Actuator::Feedback feedback_type);
    void ReportStatus(const QString& status, const Actuator::Type type);

    void ErrorThrown(const QString& err);

  protected:
    // --- Helper Functions ---

    // --- Data Members ---

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{false};  // keep to allow disabling debug-only code

    const Actuator::Type type_;
};
