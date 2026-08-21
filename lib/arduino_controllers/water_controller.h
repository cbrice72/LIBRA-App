/******************************************************************************
 * @file   water_controller.h
 * @brief  Water Arduino (counterweight system) device management; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <optional>

// Other Library Headers
#include <QByteArray>  // Qt::Core

// Project Headers
#include "arduino_defs.h"
#include "generic_serial_device.h"

/**
 * @brief Controls the fluid system (counterweight in/out) via serial connection.
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

  signals:
    // --- Arduino Updates ---

    void ReportStatus(const Water::Side& side, const Water::State& state);

  protected:
    void OnUpdate() override;

  private:
    // --- Helper Functions ---

    void ClearCommand(Water::Side side = Water::Side::kAll);
    void ModifyCommand(uint8_t new_bits);
    void MapTorqueToCommand(const double& torque_dir);

    void SendStatus(Water::Side side);

    // --- Data Members ---

    bool auto_comp_en_{false};
    QByteArray water_cmd_;  // only 4 bits used
};
