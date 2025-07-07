/******************************************************************************
 * @file   arduino_manager.h
 * @brief  QSerialPort-based Arduino device management; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QByteArray>   // Qt::Core
#include <QObject>      // Qt::Core
#include <QSerialPort>  // Qt::SerialPort
#include <QTimer>       // Qt::Core

// Project Headers
#include "arduino_defs.h"
#include "logger.h"

/**
 * @brief Manages multiple serial Arduino devices using `QTimer`-based updates.
 *
 * @note Unlike `CameraManager`, this class uses `QTimer`s to asynchronously
 *       communicate with its device(s). Therefore, it follows the same
 *       command/feedback pattern as the `QThread`-based `HebiThread` and
 *       `EposThread` classes.
 *
 * @todo Delete below notes once SerialServo rename is complete.
 * @note "Manip" = SerialServo, in charge of LIBRA-I's 3-servo manipulator.
 * @note "Water" = SerialWater, in charge of the fluid system (water in/out).
 *
 * @see CameraManager HebiThread EposThread
 */
class ArduinoManager : public QObject {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit ArduinoManager(QObject* parent, const bool& debug_mode);
    ~ArduinoManager() override;

    // For use in "Force" SetWaterCommand() calls
    static constexpr double kFill = -M_PI / 2.0;  // unused
    static constexpr double kDrain = M_PI / 2.0;

  public slots:

    void SetDebugMode(const bool& enabled) {
        debug_mode_ = enabled;
        logger_->SetDebugMode(debug_mode_);
    };

    // --- Arduino Commands ---

#if LIBRA_VERSION == 1
    void ConnectManip(QString port_name);
    void DisconnectManip();
    void SetManipCommand(const double& arm_pitch, const double& target_pan,
                         const double& target_tilt, const bool& move_slow);
#endif

    void ConnectWater(QString port_name);
    void DisconnectWater();
    void SetWaterState(const bool& enabled);
    void SetWaterCommand(double torque_dir);

  signals:
    // --- Arduino Updates ---

    void ErrorThrown(const QString& err);

#if LIBRA_VERSION == 1
    void ManipConnected(const bool& connected);
    void ReportPosition(const double& base, const double& pan,
                        const double& tilt);
#endif
    void WaterConnected(const bool& connected);
    void ReportWaterStatus(const Water::Side& side, const Water::State& state);

  private slots:
    // --- Timer Management ---

    void UpdateDevices();
    void AttemptReconnects();

  private:
    // --- Helper Functions ---

    void SendWaterStatus(Water::Side side);

    void RefreshUpdateTimerState();
#if LIBRA_VERSION == 1
    void UpdateManip();
#endif
    void UpdateWater();

    // --- Data Members ---

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{false};

    // Serial port management
    QSerialPort* ser_servo_{nullptr};
    QSerialPort* ser_water_{nullptr};

    // Timer management
    QTimer* update_timer_{nullptr};
    static constexpr int kUpdateIntervalMs = 500;  // 2 Hz

    QTimer* reconnect_timer_{nullptr};
    static constexpr int kReconnectIntervalMs = 3000;  // 3 sec
#if LIBRA_VERSION == 1
    bool manip_needs_reconnect_{false};
#endif
    bool water_needs_reconnect_{false};

#if LIBRA_VERSION == 1
    // Manip state
    std::array<double, 3> m_current_pos_{0};
    std::array<double, 3> m_target_pos_{0};
    std::array<int, 3> m_slow_direction_{0};
#endif

    // Water state
    bool water_en_{false};
    QByteArray water_cmd_;  // only 4 bits used
};
