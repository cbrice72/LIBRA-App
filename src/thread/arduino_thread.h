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
#include <QSerialPort>  // Qt::SerialPort
#include <QThread>      // Qt::Core

// Project Headers
#include "logger.h"

/**
 * @brief Operational state of the pump(s).
 */
enum PumpState { kEnable = 0, kDisable, kForceDrain };

/**
 * @brief Lightweight manager class for the SerialWater and SerialServo
 * arduinos.
 */
class ArduinoThread : public QThread {
  public:
    explicit ArduinoThread(QObject* parent, const bool& debug_mode);
    ~ArduinoThread() override;

  public slots:

    void SetDebugMode(const bool& enabled) {
        logger_->SetDebugMode(enabled);
    };

    // --- Arduino Commands ---

    void ConnectManip();
    void DisconnectManip();
    void CommandManip(const double& arm_pitch, const double& target_pan,
                      const double& target_tilt, const bool& move_slow);

    void ConnectPump();
    void DisconnectPump();
    void CommandPump(const PumpState& state);

  signals:
    // --- Arduino Updates ---

    void ManipConnected(const bool& enabled);
    void PumpConnected(const bool& enabled);

  private:
    void run() override;

    // --- Helper Functions ---

    // --- Data Members ---

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{false};

    QSerialPort* ser_servo_{nullptr};
    QSerialPort* ser_water_{nullptr};
};