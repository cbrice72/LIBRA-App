/******************************************************************************
 * @file   arduino_thread.h
 * @brief  Basic SerialWater and SerialServo management; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "arduino_thread.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDebug>           // Qt::Core
#include <QSerialPortInfo>  // Qt::SerialPort

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Thread Overrides
 * !Manipulator Commands (slots)
 * !Pump Commands (slots)
 */

constexpr uint8_t kManipSlowSpeed = 1.5;   // deg/s, arbitrary
constexpr uint8_t kManipUpdateSpeed = 60;  // Hz, assumed (see note below)

// NOTE: the old LIBRA-I control app calculated the "Slow" speed using arbitrary
//       "magic" numbers. I think this results in a nice speed, though, so I try
//       to make sense of it using the constants defined above. For reference,
//       the servos are rated at a speed of 60 deg / 0.18 sec (at 5.0 V).

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {}  // namespace

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param debug_mode Whether verbose debug text should be output
 */
ArduinoThread::ArduinoThread(QObject* parent, const bool& debug_mode)
    : QThread(parent),
      debug_mode_(debug_mode),
      ser_servo_(new QSerialPort(this)),
      ser_water_(new QSerialPort(this)) {
    // Initialize the logger
    logger_ = std::make_unique<QtLogger>(debug_mode_);
}

/**
 * @brief Standard destructor.
 */
ArduinoThread::~ArduinoThread() {
    // These calls to `Disconnect*()` simply close the serial port connections
    DisconnectManip();
    DisconnectPump();

    logger_->Debug("Cleaned up ArduinoThread");
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main arduino connection manager loop.
 */
void ArduinoThread::run() {
    if (debug_mode_) {
        logger_->Debug("Initialized ArduinoThread");
    }

    // Initialize thread variables for efficiency
    //   (none)

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // TODO: implementation

        // Update continuous log
        /*
        if (something) {
            // Timestamp
            continuous_log_ << util::GetTimestampStr() + ",,";

            // Actuator info
            for (auto i = 0; i < kHebiFeedbackCount; i++) {
                continuous_log_ << arm_current_.at(i) << ",,";
            }

            // Fluid system info
            for (auto i = 0; i < kFluidStateCount; i++) {
                continuous_log_ << ((cmd_water & (1 << (3 - i))) ? 1 : 0)
                                << ",";
            }
            continuous_log_ << ",";

            // Camera actuator info
            // TODO: use kCameraNodeCount
            continuous_log_ << manip_pos_.at(0) << "," << manip_pos_.at(1)
                            << "," << manip_pos_.at(2);

            // Flush the current line
            continuous_log_ << std::endl;
        }
        */

        QThread::msleep(100);  // update 10 times/second (theoretically)
    }
}

//------------------------------------------------------------------------------
// !Manipulator Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish a connection to the SerialServo Arduino.
 */
void ArduinoThread::ConnectManip() {
    // If there is already an active connection, gracefully terminate it
    DisconnectManip();

    // Check if device is connected at default COM port (COM4)
    bool found = false;
    foreach (const QSerialPortInfo& info, QSerialPortInfo::availablePorts()) {
        if (info.portName() == "COM4") {
            found = true;
            break;
        }
        if (debug_mode_) {
            // Enumerate available serial ports
            logger_->Debug("Found SerialPort with following metadata"
                           + "\n  Port: " + info.portName()
                           + "\n  Description: " + info.description()
                           + "\n  Manufacturer: " + info.manufacturer() + "\n");

            // TODO: query user to pick one and continue in function
            return;  // temporary
        }
    }

    if (!found) {
        logger_->Error("SerialServo (COM4) not found!");
        return;
    }

    // Set port options
    ser_servo_->setPortName("COM4");  // hard-coded, see above TODO
    ser_servo_->setBaudRate(QSerialPort::Baud115200);
    ser_servo_->setDataBits(QSerialPort::Data8);
    ser_servo_->setParity(QSerialPort::NoParity);
    ser_servo_->setStopBits(QSerialPort::OneStop);
    ser_servo_->setFlowControl(QSerialPort::NoFlowControl);

    // Only continue if "open" was successful
    if (!ser_servo_->open(QIODevice::ReadWrite)) {
        logger_->Debug("Failed to open port: COM4!");
        QMessageBox::critical(this, tr("Error"), ser_servo_->errorString());
        return;
    }

    emit ManipConnected(true);
}

/**
 * @brief Terminates the active connection.
 */
void ArduinoThread::DisconnectManip() {
    if (ser_servo_->isOpen()) {
        ser_servo_->close();
    }

    emit ManipConnected(false);
}

/**
 * @brief TODO: documentation.
 *
 * @param arm_pitch ...
 * @param target_pan ...
 * @param target_tilt ...
 * @param move_slow ...
 */
void ArduinoThread::CommandManip(const double& arm_pitch,
                                 const double& target_pan,
                                 const double& target_tilt,
                                 const bool& move_slow) {
    if (!ser_servo_->isOpen()) {
        logger_->Warn("SerialServo not connected!");
        return;
    }

    if (move_slow) {
        // Move manipulator servos at a leisurely pace
        auto target_pan = ui_->sb_manip_pan->value();
        auto target_tilt = ui_->sb_manip_tilt->value();

        auto pan_dir = (target_pan > current_pan)   ? 1
                       : (target_pan < current_pan) ? -1
                                                    : 0;
        auto tilt_dir = (target_tilt > current_tilt)   ? 1
                        : (target_tilt < current_tilt) ? -1
                                                       : 0;

        current_pan += pan_dir * kManipSlowSpeed / kManipUpdateSpeed;
        current_tilt += tilt_dir * kManipSlowSpeed / kManipUpdateSpeed;
    } else {
        // Move manipulator servos at maximum speed (near-instant)
        current_pan = target_pan;
        current_tilt = target_tilt;
    }
}

//------------------------------------------------------------------------------
// !Pump Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish a connection to the SerialWater Arduino.
 */
void ArduinoThread::ConnectPump() {
    // If there is already an active connection, gracefully terminate it
    DisconnectPump();

    // Check if device is connected at default COM port (COM3)
    bool found = false;
    foreach (const QSerialPortInfo& info, QSerialPortInfo::availablePorts()) {
        if (info.portName() == "COM3") {
            found = true;
            break;
        }
        if (debug_mode_) {
            // Enumerate available serial ports
            logger_->Debug("Found SerialPort with following metadata"
                           + "\n  Port: " + info.portName()
                           + "\n  Description: " + info.description()
                           + "\n  Manufacturer: " + info.manufacturer() + "\n");

            // TODO: query user to pick one and continue in function
            return;  // temporary
        }
    }

    if (!found) {
        logger_->Error("SerialWater (COM3) not found!");
        return;
    }

    // Set port options
    ser_water_->setPortName("COM3");  // hard-coded, see above TODO
    ser_water_->setBaudRate(QSerialPort::Baud115200);
    ser_water_->setDataBits(QSerialPort::Data8);
    ser_water_->setParity(QSerialPort::NoParity);
    ser_water_->setStopBits(QSerialPort::OneStop);
    ser_water_->setFlowControl(QSerialPort::NoFlowControl);

    // Only continue if "open" was successful
    if (!ser_water_->open(QIODevice::ReadWrite)) {
        logger_->Error("Failed to open port: COM3!");
        QMessageBox::critical(this, tr("Error"), ser_water_->errorString());
        return;
    }

    emit PumpConnected(true);
}

/**
 * @brief Terminates the active connection.
 */
void ArduinoThread::DisconnectPump() {
    if (ser_water_->isOpen()) {
        ser_water_->close();
    }

    emit PumpConnected(false);
}

/**
 * @brief TODO: documentation.
 */
void ArduinoThread::CommandPump(const PumpState& state) {
    if (!ser_water_->isOpen()) {
        logger_->Warn("SerialWater not connected!");
        return;
    }

    logger_->Warn("Pump control not yet implemented!");

    // TODO: implementation (adapt code below)
    switch (state) {
        case PumpState::kEnable:
            // Enable operation of pump(s)
            /*
            water_en_ = true;
            water_mode_ = WaterMode::kStandby;
            */
            break;
        case PumpState::kDisable:
            /*
            water_en_ = false;
            water_mode_ = WaterMode::kStandby;
            */
            break;
        case PumpState::kForceDrain:
            /*
            water_en_ = true;
            water_mode_ = WaterMode::kDrain;
            */
            break;
        default:
    }
}
