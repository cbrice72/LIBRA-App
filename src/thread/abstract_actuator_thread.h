/******************************************************************************
 * @file   abstract_actuator_thread.h
 * @brief  Abstract control class for LIBRA actuators; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <atomic>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

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
    AbstractActuatorThread() = delete;

    explicit AbstractActuatorThread(QObject* parent, const bool& debug_mode,
                                    const Actuator::Type& type)
        : QThread(parent), debug_mode_(debug_mode), type_(type) {}

    ~AbstractActuatorThread() override = default;

    // --- Getters & Setters ---

    /**
     * @brief Gets the actuator(s) last reported active state.
     */
    [[nodiscard]] bool IsMoving() const {
        return moving_;
    }

  public slots:

    void SetDebugMode(const bool& enabled) {
        debug_mode_ = enabled;
        logger_->SetDebugMode(debug_mode_);
    }

    // --- Actuator Commands ---

    virtual void Connect() = 0;
    virtual void Disconnect() = 0;

    virtual void SetTarget(const std::vector<double>& deg) = 0;
    virtual void Stop() = 0;

  signals:
    // --- Actuator Updates ---

    void Connected(const bool& connected);

    void ReportFeedback(const std::unordered_map<Joint::Name, double>& feedbacks,
                        const Actuator::Feedback& feedback_type);
    void ReportTelemetry(const QString& telemetry, const Actuator::Type& type);
    void MovementStateChanged(const bool& moving);

    void ErrorThrown(const QString& err);

  protected:
    // --- Helper Functions ---

    /**
     * @brief Tells listeners when the actuator(s) start or stop moving.
     *
     * @param moving Whether a move is running/was requested
     *
     * @note This should only be called from `run()` to avoid sending messages
     *       out of order. Only if `run()` isn't actively reporting anything
     *       (e.g., after `Disconnect()`) should this be called from outside.
     */
    void ReportMovement(const bool& moving) {
        if (moving_.exchange(moving) != moving) {
            emit MovementStateChanged(moving);
        }
    }

    /**
     * @brief Appends an aligned row of numbers (one per actuator) for telemetry
     *        display.
     *
     * @param ss The output stream being built
     * @param label Row label
     * @param values One value per actuator
     * @param unit Unit of the values, if any
     */
    static void AddTelemetryRow(std::ostringstream& ss,
                                const std::string& label,
                                const std::vector<double>& values,
                                const std::string& unit = "") {
        ss << std::left << std::setw(kTelemetryLabelWidth) << label
           << std::right << std::fixed
           << std::setprecision(kTelemetryPrecision);
        for (const double& val : values) {
            ss << std::setw(kTelemetryValueWidth) << val;
        }
        if (!unit.empty()) {
            ss << " " << unit;
        }
        ss << "\n";
    }

    /**
     * @brief Appends an aligned row of text (e.g., names, states) for telemetry
     *        display.
     *
     * @param ss The output stream being built
     * @param label Row label
     * @param values One entry per actuator
     */
    static void AddTelemetryTextRow(std::ostringstream& ss,
                                    const std::string& label,
                                    const std::vector<std::string>& values) {
        ss << std::left << std::setw(kTelemetryLabelWidth) << label
           << std::right;
        for (const auto& val : values) {
            ss << std::setw(kTelemetryValueWidth) << val;
        }
        ss << "\n";
    }

    // --- Data Members ---

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{false};

    const Actuator::Type type_;

    std::atomic<bool> move_requested_{false};
    std::atomic<bool> moving_{false};  // last reported movement state

    static constexpr int kTelemetryLabelWidth = 20;  // longest label, +1
    static constexpr int kTelemetryValueWidth = 8;  // "-123.45" or "[Pitch]", +1
    static constexpr int kTelemetryPrecision = 2;   // i.e., "0.01"
};
