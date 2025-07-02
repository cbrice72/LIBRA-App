/******************************************************************************
 * @file   arduino_thread.h
 * @brief  Basic SerialWater and SerialServo management; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QByteArray>   // Qt::Core
#include <QSerialPort>  // Qt::SerialPort
#include <QThread>      // Qt::Core

// Project Headers
#include "arduino_defs.h"
#include "logger.h"

/**
 * @brief Lightweight manager class for the SerialWater and SerialServo
 * arduinos.
 */
class ArduinoThread : public QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit ArduinoThread(QObject* parent, const bool& debug_mode);
    ~ArduinoThread() override;

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

#if LIBRA_VERSION == 1
    void ManipConnected(const bool& connected);
    void ReportPosition(const double& base, const double& pan,
                        const double& tilt);
#endif
    void WaterConnected(const bool& connected);
    void ReportWaterStatus(const Water::Side& side, const Water::State& state);

    void ErrorThrown(const QString& err);

  private:
    void run() override;

    // --- Helper Functions ---

    void SendWaterStatus(Water::Side side);

    // --- Data Members ---

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{false};  // keep to allow disabling debug-only

    QSerialPort* ser_servo_{nullptr};
    QSerialPort* ser_water_{nullptr};

    std::array<double, 3> m_current_pos_{0};
    std::array<double, 3> m_target_pos_{0};
    std::array<int, 3> m_slow_direction_{0};

    bool water_en_{false};
    QByteArray water_cmd_;  // only 4 bits used
};