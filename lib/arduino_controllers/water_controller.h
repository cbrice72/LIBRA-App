/******************************************************************************
 * @file   water_controller.h
 * @brief  Water Arduino (counterweight system) device management; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <array>
#include <optional>

// Other Library Headers
#include <QByteArray>     // Qt::Core
#include <QElapsedTimer>  // Qt::Core
#include <QTimer>         // Qt::Core

// Project Headers
#include "arduino_defs.h"
#include "generic_serial_device.h"

/**
 * @brief Controls the fluid system (counterweight in/out) via serial connection
 *        and tracks each CW's volume from its valve state and flow rate.
 */
class WaterController : public GenericSerialDevice {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit WaterController(QObject* parent, const bool& debug_mode);
    ~WaterController() override = default;

  public slots:
    // --- Arduino Commands ---

    void EnableAutoTorqueComp(const bool& enabled);
    void UpdateTorqueFeedback(const std::optional<double>& torque_dir);

    void ForceCommand(const Water::Side& side, const Water::State& state);

    // --- Volume Tracking ---

    void UpdateFlowRate(const Water::Side& side, const double& flow);
    void OverrideVolume(const Water::Side& side, const double& volume);

  signals:
    // --- Arduino Updates ---

    void ReportStatus(const Water::Side& side, const Water::State& state);
    void ReportVolume(const Water::Side& side, const double& volume,
                      const double& flow);
    void TankLimitReached(const Water::Side& side, const bool& full);

  protected:
    void OnUpdate() override;

  private:
    /**
     * @brief Estimated state of one CW.
     */
    struct Tank {
        Water::State state{Water::State::kStopped};  // last sent valve state
        double volume{0.0};                          // V (L)
        double flow{0.0};                            // last reported V̇ (L/s)
        bool unreported{true};                       // force next report
        double fill_rate{Water::kFillRate};          // (L/s)
        double drain_rate{Water::kDrainRate};        // (L/s)
    };

    // --- Helper Functions ---

    void ClearCommand(const Water::Side& side = Water::Side::kAll);
    void ModifyCommand(uint8_t new_bits);
    void MapTorqueToCommand(const double& torque_dir);

    void SendStatus(const Water::Side& side);
    void TrackVolume();

    // --- Data Members ---

    bool auto_comp_en_{false};
    QByteArray water_cmd_;  // only 4 bits used

    std::array<Tank, Water::kSideCount> tanks_;
    QTimer* volume_timer_{nullptr};
    QElapsedTimer volume_clock_;
};
