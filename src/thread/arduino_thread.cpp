/******************************************************************************
 * @file   arduino_thread.h
 * @brief  Basic SerialWater and SerialServo management; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "arduino_thread.h"

// C++ Standard Library Headers
#include <bitset>
#include <sstream>

// Other Library Headers
#include <QDebug>           // Qt::Core
#include <QSerialPortInfo>  // Qt::SerialPort

// Project Headers
#include "qt_logger.h"

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Class Management
 * !Class Helpers
 * !Thread Overrides
 * !Manipulator Commands (slots)
 * !Water Commands (slots)
 */

#if LIBRA_VERSION == 1
// Convenience constants for manipulator control
constexpr uint8_t kPitch = 0;
constexpr uint8_t kPan = 1;
constexpr uint8_t kTilt = 2;

// Manipulator control
constexpr double kManipSlowSpeed = 1.5;   // deg/s, arbitrary
constexpr double kManipUpdateSpeed = 60;  // Hz, assumed (see note below)
constexpr double kManipSlowMultiplier = kManipSlowSpeed / kManipUpdateSpeed;
#endif

// NOTE: the old LIBRA-I control app calculated the "Slow" speed using
// arbitrary
//       "magic" numbers. I think this results in a nice speed, though, so I
//       try to make sense of it using the constants defined above. For
//       reference, the servos are rated at a speed of 60 deg / 0.18 sec
//       (at 5.0 V).

// Bit positions for water_cmd_ (see usage in following constexpr block)
constexpr uint8_t kAIn = 0b1000;
constexpr uint8_t kBIn = 0b0100;
constexpr uint8_t kAOut = 0b0010;
constexpr uint8_t kBOut = 0b0001;

// Water commands based on torque direction (where North = PI/2 = forward)
constexpr uint8_t kWestCmd = kAIn | kBOut;    // 0b1001
constexpr uint8_t kNorthWestCmd = kBOut;      // 0b0001
constexpr uint8_t kNorthCmd = kAOut | kBOut;  // 0b0011
constexpr uint8_t kNorthEastCmd = kAOut;      // 0b0010
constexpr uint8_t kEastCmd = kBIn | kAOut;    // 0b0110
constexpr uint8_t kSouthEastCmd = kBIn;       // 0b0100
constexpr uint8_t kSouthCmd = kAIn | kBIn;    // 0b1100
constexpr uint8_t kSouthWestCmd = kAIn;       // 0b1000

// NOLINTBEGIN(readability-identifier-naming)
// Angular thresholds for direction determination
constexpr double k1_8Pi = M_PI / 8.0;
constexpr double k3_8Pi = M_PI * 3.0 / 8.0;
constexpr double k5_8Pi = M_PI * 5.0 / 8.0;
constexpr double k7_8Pi = M_PI * 7.0 / 8.0;

// NOLINTEND(readability-identifier-naming)

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Converts a QByteArray to QString in binary format (for printing),
 *        since Qt doesn't seem to support this natively.
 *
 * @param bytes The QByteArray to convert
 * @return std::string A representation of the input QBitArray in binary format
 */
std::string BytesToStr(QByteArray bytes) {
    std::string str;

    for (const auto& byte : bytes)
        for (auto i = 7; i >= 0; --i) {  // hard-coded range of 0-7 = one byte
            str += (byte & (1 << i)) ? '1' : '0';
        }

    return str;
}

}  // namespace

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
#if LIBRA_VERSION == 1
    DisconnectManip();
#endif
    DisconnectWater();

    logger_->Debug("Cleaned up ArduinoThread");
}

//------------------------------------------------------------------------------
// !Class Helpers
//------------------------------------------------------------------------------

/**
 * @brief Parses the active water command based on the specified side and
 *        reports its status.
 *
 * @param side The side of the fluid system to check
 * @return Water::State The state of the specified fluid system
 */
void ArduinoThread::SendWaterStatus(Water::Side side) {
    auto state = Water::State::kStopped;

    if (ser_water_->isOpen() && !water_cmd_.isEmpty()) {
        // Parse command to get corresponding state
        uint8_t command = static_cast<uint8_t>(water_cmd_[0]);
        switch (side) {
            case Water::Side::kA:
                if ((command & kAIn) != 0) {
                    state = Water::State::kFilling;
                } else if ((command & kAOut) != 0) {
                    state = Water::State::kDraining;
                }
                break;
#if LIBRA_VERSION == 1
            case Water::Side::kB:
                if ((command & kBIn) != 0) {
                    state = Water::State::kFilling;
                } else if ((command & kBOut) != 0) {
                    state = Water::State::kDraining;
                }
                break;
#endif
            default:
                // Do nothing (since state is initialized at the top)
                break;
        }
    }

    emit ReportWaterStatus(side, state);
}

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main arduino connection manager loop.
 */
void ArduinoThread::run() {
    logger_->Debug("Initialized ArduinoThread");

    // Initialize thread variables for efficiency
    QString servo_cmd;

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
#if LIBRA_VERSION == 1
        // SerialServo (manipulator)
        if (ser_servo_->isOpen()) {
            // Correct for LIBRA-I arm pitch
            m_current_pos_.at(kPitch) = (m_current_pos_.at(kPitch) <= 0)
                                            ? -m_current_pos_.at(kPitch)
                                            : 180 - m_current_pos_.at(kPitch);

            // If a "slow" movement command was received, calculate the next step
            for (auto i = 1; i < 3; ++i) {
                if (m_slow_direction_.at(i) != 0) {
                    // Increase by small arbitrary amount
                    m_current_pos_.at(i) += m_slow_direction_.at(i)
                                            * kManipSlowMultiplier;

                    // If target has been reached, reset relevant variables
                    // NOTE: there is no check for moving in a "negative"
                    //       direction because the servos do not take negative
                    //       values. This conversion is handled in the file
                    //       `arduino/servo_arduino/servo_arduino.ino`, function
                    //       `mapfloat()`.
                    if ((m_current_pos_.at(i) > m_target_pos_.at(i))
                        && (m_slow_direction_.at(i) == 1)) {
                        m_current_pos_.at(i) = m_target_pos_.at(i);
                        m_slow_direction_.at(i) = 0;
                    }
                }
            }

            // Prepare and send command: "<PITCH> <PAN> <TILT>\n"
            servo_cmd = QString::asprintf("%.1f %.1f %.1f\n",
                                          m_current_pos_.at(kPitch),
                                          m_current_pos_.at(kPan),
                                          m_current_pos_.at(kTilt));
            ser_servo_->write(servo_cmd.toUtf8());

            emit ReportPosition(m_current_pos_.at(kPitch),
                                m_current_pos_.at(kPan),
                                m_current_pos_.at(kTilt));
        }
#endif

        // SerialWater
        if (ser_water_->isOpen()) {
            ser_water_->write(water_cmd_);

            SendWaterStatus(Water::Side::kA);  // emits ReportWaterStatus
#if LIBRA_VERSION == 1
            SendWaterStatus(Water::Side::kB);  // "
#endif
        }
    }

    QThread::msleep(500);  // update 2 times/second (theoretically)
}

//------------------------------------------------------------------------------
// !Manipulator Commands (slots)
//------------------------------------------------------------------------------

#if LIBRA_VERSION == 1
/**
 * @brief Attempts to establish a connection to the SerialServo Arduino.
 */
void ArduinoThread::ConnectManip(QString port_name) {
    // If there is already an active connection, gracefully terminate it
    DisconnectManip();

    // Set port options
    ser_servo_->setPortName(port_name);
    ser_servo_->setBaudRate(QSerialPort::Baud115200);
    ser_servo_->setDataBits(QSerialPort::Data8);
    ser_servo_->setParity(QSerialPort::NoParity);
    ser_servo_->setStopBits(QSerialPort::OneStop);
    ser_servo_->setFlowControl(QSerialPort::NoFlowControl);

    // Only continue if "open" was successful
    if (!ser_servo_->open(QIODevice::ReadWrite)) {
        emit ErrorThrown("Manip - Failed to open port: COM4!\n"
                         + ser_servo_->errorString());
        return;
    }

    emit ManipConnected(true);

    logger_->Info("Manip - Connected to device at: " + port_name.toStdString());
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
 * @param arm_pitch Angle of LIBRA-I arm pitch joint "J3"
 * @param target_pan Target yaw angle
 * @param target_tilt Target pitch angle
 * @param move_slow Whether to use a slower, controlled trajectory
 */
void ArduinoThread::SetManipCommand(const double& arm_pitch,
                                    const double& target_pan,
                                    const double& target_tilt,
                                    const bool& move_slow) {
    if (!ser_servo_->isOpen()) {
        logger_->Warn("SerialServo not connected!");
        return;
    }

    m_current_pos_.at(kPitch) = arm_pitch;

    if (move_slow) {
        // Move manipulator servos at a leisurely pace
        m_slow_direction_.at(kPan) = (target_pan > m_current_pos_.at(kPan)) ? 1
                                     : (target_pan < m_current_pos_.at(kPan))
                                         ? -1
                                         : 0;
        m_slow_direction_.at(kTilt) = (target_tilt > m_current_pos_.at(kTilt))
                                          ? 1
                                      : (target_tilt < m_current_pos_.at(kTilt))
                                          ? -1
                                          : 0;
    } else {
        // Move manipulator servos at maximum speed (near-instant)
        m_current_pos_.at(kPan) = target_pan;
        m_current_pos_.at(kTilt) = target_tilt;
    }

    logger_->Debug("Set manip targets to "
                   + std::to_string(m_current_pos_.at(kPitch)) + " "
                   + std::to_string(m_current_pos_.at(kPan)) + " "
                   + std::to_string(m_current_pos_.at(kTilt)) + " deg");
}
#endif

//------------------------------------------------------------------------------
// !Water Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish a connection to the SerialWater Arduino.
 */
void ArduinoThread::ConnectWater(QString port_name) {
    // If there is already an active connection, gracefully terminate it
    DisconnectWater();

    // Set port options
    ser_water_->setPortName(port_name);
    ser_water_->setBaudRate(QSerialPort::Baud115200);
    ser_water_->setDataBits(QSerialPort::Data8);
    ser_water_->setParity(QSerialPort::NoParity);
    ser_water_->setStopBits(QSerialPort::OneStop);
    ser_water_->setFlowControl(QSerialPort::NoFlowControl);

    // Only continue if "open" was successful
    if (!ser_water_->open(QIODevice::ReadWrite)) {
        emit ErrorThrown("Water - Failed to open port: COM3!\n"
                         + ser_water_->errorString());
        return;
    }

    emit WaterConnected(true);

    logger_->Info("Water - Connected to device at: " + port_name.toStdString());
}

/**
 * @brief Terminates the active connection.
 */
void ArduinoThread::DisconnectWater() {
    if (ser_water_->isOpen()) {
        ser_water_->close();
    }

    emit WaterConnected(false);
}

/**
 * @brief Sets the operational state of the fluid system.
 */
void ArduinoThread::SetWaterState(const bool& enabled) {
    water_en_ = enabled;

    if (!water_en_) {
        water_cmd_.clear();  // clear any active water commands
    }

    logger_->Debug("Fluid system "
                   + std::string(enabled ? "enabled" : "disabled"));
}

/**
 * @brief Calculates counterweight fill/drain command based on torque acting
 *        upon central joint.
 *
 * @param torque_dir Direction of torque feedback, in radians (range: [-PI, PI])
 *
 * @note Command bit field "0b1234" -> 1: A_IN | 2: B_IN | 3: A_OUT | 4: B_OUT
 *
 * @note This function is only triggered under two situations:
 *   1) `HebiThread` emits the `ReportArmTorque` signal, which means
 *        automatic torque compensation is enabled.
 *   2) `MainWindow` emits the `CommandWater` signal, which in this case is only
 *        used to force fill or drain the counterweight(s).
 *
 * @see hebi_thread
 */
void ArduinoThread::SetWaterCommand(double torque_dir) {
    if (!ser_water_->isOpen()) {
        logger_->Warn("SerialWater not connected!");
        return;
    }

    uint8_t command;

#if LIBRA_VERSION == 1
    if (torque_dir > k7_8Pi || -k7_8Pi >= torque_dir) {  // W
        command = kWestCmd;
    } else if (torque_dir > k5_8Pi) {  // NW
        command = kNorthWestCmd;
    } else if (torque_dir > k3_8Pi) {  // N
        command = kNorthCmd;
    } else if (torque_dir > k1_8Pi) {  // NE
        command = kNorthEastCmd;
    } else if (torque_dir > -k1_8Pi) {  // E
        command = kEastCmd;
    } else if (torque_dir > -k3_8Pi) {  // SE
        command = kSouthEastCmd;
    } else if (torque_dir > -k5_8Pi) {  // S
        command = kSouthCmd;
    } else {  // SW
        command = kSouthWestCmd;
    }
#elif LIBRA_VERSION == 2
    if (torque_dir == 0) {  // N
        command = kNorthCmd;
    } else {  // S
        command = kSouthCmd;
    }
#endif

    water_cmd_ = QByteArray(1, static_cast<char>(command));

    logger_->Debug("Set water command to " + BytesToStr(water_cmd_)
                   + " (A_IN | B_IN | A_OUT | B_OUT)");
}
