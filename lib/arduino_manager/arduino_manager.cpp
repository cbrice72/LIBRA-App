/******************************************************************************
 * @file   arduino_manager.cpp
 * @brief  QSerialPort-based Arduino device management; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "arduino_manager.h"

// C++ Standard Library Headers
#include <bitset>
#include <iostream>  // TODO: remove once BytesToStr() TODOs are fulfilled
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
 * !Timer Management
 * !Timer Helpers
 * !Water Commands (slots)
 * !Manipulator Commands (slots)
 */

// Bit Positions for water_cmd_ (see usage in following constexpr block)

constexpr uint8_t kAIn = 0b1000;
constexpr uint8_t kBIn = 0b0100;
constexpr uint8_t kAOut = 0b0010;
constexpr uint8_t kBOut = 0b0001;

// Water Commands based on Torque Direction (where North = PI/2 = forward)

constexpr uint8_t kWestCmd = kAIn | kBOut;    // 0b1001
constexpr uint8_t kNorthWestCmd = kBOut;      // 0b0001
constexpr uint8_t kNorthCmd = kAOut | kBOut;  // 0b0011
constexpr uint8_t kNorthEastCmd = kAOut;      // 0b0010
constexpr uint8_t kEastCmd = kBIn | kAOut;    // 0b0110
constexpr uint8_t kSouthEastCmd = kBIn;       // 0b0100
constexpr uint8_t kSouthCmd = kAIn | kBIn;    // 0b1100
constexpr uint8_t kSouthWestCmd = kAIn;       // 0b1000

// NOLINTBEGIN(readability-identifier-naming)
// Angular Thresholds for Direction Determination

constexpr double k1_8Pi = M_PI / 8.0;
constexpr double k3_8Pi = M_PI * 3.0 / 8.0;
constexpr double k5_8Pi = M_PI * 5.0 / 8.0;
constexpr double k7_8Pi = M_PI * 7.0 / 8.0;

// NOLINTEND(readability-identifier-naming)

#if LIBRA_VERSION == 1
// Convenience Indexes for Manipulator Control

constexpr uint8_t kPitch = 0;
constexpr uint8_t kPan = 1;
constexpr uint8_t kTilt = 2;

// Manipulator Control

constexpr double kManipSlowSpeed = 1.5;   // deg/s, arbitrary
constexpr double kManipUpdateSpeed = 60;  // Hz, assumed (see note below)
constexpr double kManipSlowMultiplier = kManipSlowSpeed / kManipUpdateSpeed;
// NOTE: the old LIBRA-I control app calculated the "Slow" speed using arbitrary
//       "magic" numbers. I think this results in a nice speed, though, so I try
//       to make sense of it using the constants defined above. For reference,
//       the servos are rated at a speed of 60 deg / 0.18 sec (at 5.0 V).
#endif

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
std::string BytesToStr(const QByteArray& bytes) {
    std::string str;

    // TODO: check the result of this block against the new one before deleting it
    // ---
    std::string old_str;
    for (const auto& byte : bytes) {
        for (auto i = 7; i >= 0; --i) {  // hard-coded range of 0-7 = one byte
            old_str += (byte & (1 << i)) ? '1' : '0';
        }
    }
    // ---

    for (auto byte : bytes) {
        str += std::bitset<8>(static_cast<unsigned char>(byte)).to_string();
    }

    // TODO: temporary (see above TODO)
    std::cout << "ArduinoManager - Old vs. New BytesToStr(): " << old_str
              << " | " << str << std::endl;

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
 * @param debug_mode Whether to output verbose debug text
 */
ArduinoManager::ArduinoManager(QObject* parent, const bool& debug_mode)
    : QObject(parent),
      debug_mode_(debug_mode),
      ser_water_(new QSerialPort(this)),
      ser_manip_(new QSerialPort(this)) {
    // Initialize the logger
    logger_ = std::make_unique<QtLogger>(debug_mode_);

    // Set up timers
    update_timer_ = new QTimer(this);
    reconnect_timer_ = new QTimer(this);

    // Connect timer signals
    connect(update_timer_, &QTimer::timeout, this,
            &ArduinoManager::UpdateDevices);
    connect(reconnect_timer_, &QTimer::timeout, this,
            &ArduinoManager::AttemptReconnects);

    // Configure timers
    update_timer_->setInterval(kUpdateIntervalMs);
    reconnect_timer_->setInterval(kReconnectIntervalMs);

    logger_->Debug("Initialized ArduinoManager");
}

/**
 * @brief Standard destructor.
 */
ArduinoManager::~ArduinoManager() {
    // Stop all timers
    update_timer_->stop();
    reconnect_timer_->stop();

    // These calls to Disconnect*() simply close the serial port connections
    DisconnectWater();
#if LIBRA_VERSION == 1
    DisconnectManip();
#endif

    logger_->Debug("Cleaned up ArduinoManager");
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
void ArduinoManager::SendWaterStatus(Water::Side side) {
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

/**
 * @brief Starts or stops the update timer depending on whether any devices are
 *        connected.
 */
void ArduinoManager::RefreshUpdateTimerState() {
    bool any_connected = false;

    // Check if any devices are connected
    any_connected |= ser_water_->isOpen();
#if LIBRA_VERSION == 1
    any_connected |= ser_manip_->isOpen();
#endif

    if (any_connected && !update_timer_->isActive()) {
        // A device(s) has been connected, so start sending it commands
        update_timer_->start();
        logger_->Debug("ArduinoManager - Started update timer");
    } else if (!any_connected && update_timer_->isActive()) {
        // No devices are connected, so conserve resources
        update_timer_->stop();
        logger_->Debug("ArduinoManager - Stopped update timer");
    }
}

//------------------------------------------------------------------------------
// !Timer Management
//------------------------------------------------------------------------------

/**
 * @brief Comprehensive device update callback.
 */
void ArduinoManager::UpdateDevices() {
    UpdateWater();
#if LIBRA_VERSION == 1
    UpdateManip();
#endif
}

/**
 * @brief Attempts to reconnect devices, if necessary.
 */
void ArduinoManager::AttemptReconnects() {
    if (water_needs_reconnect_) {
        ConnectWater(ser_water_->portName());
    }
#if LIBRA_VERSION == 1
    if (manip_needs_reconnect_) {
        ConnectManip(ser_manip_->portName());
    }
#endif

    // Stop timer if no devices need reconnect
    if (!water_needs_reconnect_ && !manip_needs_reconnect_) {
        reconnect_timer_->stop();
        logger_->Debug("ArduinoManager - Stopped reconnect timer");
    }
}

//------------------------------------------------------------------------------
// !Timer Helpers
//------------------------------------------------------------------------------

/**
 * @brief Sends the previously-assigned command to the SerialWater Arduino.
 *
 * @see MapTorqueToWaterCommand
 */
void ArduinoManager::UpdateWater() {
    if (!ser_water_->isOpen()) {
        return;
    }

    try {
        qint64 bytes_written = ser_water_->write(water_cmd_);
        if (bytes_written == -1) {
            throw std::runtime_error("Water - Failed to write to port");
        }

        SendWaterStatus(Water::Side::kA);  // emits ReportWaterStatus
#if LIBRA_VERSION == 1
        SendWaterStatus(Water::Side::kB);  // "
#endif

    } catch (const std::exception& e) {
        emit ErrorThrown("Water - Communication error: " + QString(e.what()));

        // Close the problematic connection
        ser_water_->close();
        emit WaterConnected(false);

        // Mark for reconnection
        water_needs_reconnect_ = true;
        if (!reconnect_timer_->isActive()) {
            reconnect_timer_->start();
            logger_->Debug("Water - Started reconnection timer");
        }
    }
}

#if LIBRA_VERSION == 1
/**
 * @brief Sends the previously-assigned command to the SerialServo Arduino.
 *
 * @see SetManipCommand
 */
void ArduinoManager::UpdateManip() {
    if (!ser_manip_->isOpen()) {
        return;
    }

    try {
        // Correct for LIBRA-I arm pitch
        m_current_pos_.at(kPitch) = (m_current_pos_.at(kPitch) <= 0)
                                        ? -m_current_pos_.at(kPitch)
                                        : 180 - m_current_pos_.at(kPitch);

        // If a "slow" movement command was specified, manually calculate the
        // next step to achieve a leisurely pace
        // (NOTE: loop starts at i = 1 since Pitch is calculated separately)
        for (auto i = 1; i < 3; ++i) {
            if (m_slow_direction_.at(i) != 0) {
                // Increase by small arbitrary amount
                m_current_pos_.at(i) += m_slow_direction_.at(i)
                                        * kManipSlowMultiplier;

                // If target has been reached, reset relevant variables
                /*
                // NOTE: there is no check for moving in a "negative"
                //       direction because the servos do not take negative
                //       values. This conversion is handled in the file
                //       `arduino/servo_arduino/servo_arduino.ino`, function
                //       `mapfloat()`.
                if ((m_current_pos_.at(i) > m_target_pos_.at(i))
                    && (m_slow_direction_.at(i) == 1)) {
                */
                if ((m_slow_direction_.at(i) == 1
                     && m_current_pos_.at(i) > m_target_pos_.at(i))
                    || (m_slow_direction_.at(i) == -1
                        && m_current_pos_.at(i) < m_target_pos_.at(i))) {
                    m_current_pos_.at(i) = m_target_pos_.at(i);
                    m_slow_direction_.at(i) = 0;
                }
            }
        }

        // Prepare and send command: "<PITCH> <PAN> <TILT>\n"
        QString servo_cmd = QString::asprintf("%.1f %.1f %.1f\n",
                                              m_current_pos_.at(kPitch),
                                              m_current_pos_.at(kPan),
                                              m_current_pos_.at(kTilt));

        if (ser_manip_->write(servo_cmd.toUtf8()) == -1) {
            throw std::runtime_error("Manip - Failed to write to port");
        }

        emit ReportPosition(m_current_pos_.at(kPitch), m_current_pos_.at(kPan),
                            m_current_pos_.at(kTilt));

    } catch (const std::exception& e) {
        emit ErrorThrown("Manip - Communication error: " + QString(e.what()));

        // Close the problematic connection
        ser_manip_->close();
        emit ManipConnected(false);

        // Mark for reconnection
        manip_needs_reconnect_ = true;
        if (!reconnect_timer_->isActive()) {
            reconnect_timer_->start();
            logger_->Debug("Manip - Started reconnection timer");
        }
    }
}
#endif

//------------------------------------------------------------------------------
// !Water Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish a connection to the SerialWater Arduino.
 *
 * @param port_name The serial device address to connect to
 */
void ArduinoManager::ConnectWater(const QString& port_name) {
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
        emit ErrorThrown("Water - Failed to open port: " + port_name + "!\n"
                         + ser_water_->errorString());
        return;
    }

    logger_->Info("Water - Connected to device at: " + port_name.toStdString());
    emit WaterConnected(true);

    // If update_timer_ is stopped, restart it
    RefreshUpdateTimerState();
}

/**
 * @brief Terminates the active connection.
 */
void ArduinoManager::DisconnectWater() {
    // This function is only called intentionally, so don't attempt to reconnect
    water_needs_reconnect_ = false;

    if (ser_water_->isOpen()) {
        ser_water_->close();

        logger_->Debug("Water - Gracefully disconnected from device");
        emit WaterConnected(false);
    }

    // If update_timer_ is running, and no other devices are connected, stop it
    RefreshUpdateTimerState();
}

/**
 * @brief Sets the state of the fluid system's automatic torque compensation.
 *
 * @see MapTorqueToWaterCommand
 */
void ArduinoManager::SetAutoCompensation(const bool& enabled) {
    auto_comp_en_.store(enabled);

    if (!enabled) {
        water_cmd_.clear();  // clear any active water commands
    }

    logger_->Debug("Water - Fluid system "
                   + std::string(enabled ? "enabled" : "disabled"));
}

/**
 * @brief Determines counterweight fill/drain command based on direction of
 *        torque acting upon central joint.
 *
 * @param torque_dir Direction of torque feedback, in radians (range: [-PI, PI])
 *
 * @note Command bit field "0b1234" -> 1: A_IN | 2: B_IN | 3: A_OUT | 4: B_OUT.
 *
 * @see hebi_thread::run
 */
void ArduinoManager::MapTorqueToWaterCommand(const double& torque_dir) {
    if (!auto_comp_en_.load()) {
        // Only allow manual control (see ForceWaterCommand)
        return;
    }

    if (!ser_water_->isOpen()) {
        // Unlike other isOpen checks, do NOT log any messages here because this
        // function can be called by HebiThread multiple times per second
        return;
    }

    uint8_t command = 0;

#if LIBRA_VERSION == 1
    // Set command based on radial direction
    if (torque_dir > k7_8Pi || -k7_8Pi >= torque_dir) {
        command = kWestCmd;  // kAIn | kBOut
    } else if (torque_dir > k5_8Pi) {
        command = kNorthWestCmd;  // kBOut
    } else if (torque_dir > k3_8Pi) {
        command = kNorthCmd;  // kAOut | kBOut
    } else if (torque_dir > k1_8Pi) {
        command = kNorthEastCmd;  // kAOut
    } else if (torque_dir > -k1_8Pi) {
        command = kEastCmd;  // kBIn | kAOut
    } else if (torque_dir > -k3_8Pi) {
        command = kSouthEastCmd;  // kBIn
    } else if (torque_dir > -k5_8Pi) {
        command = kSouthCmd;  // kAIn | kBIn
    } else {
        command = kSouthWestCmd;  // kAIn
    }
#elif LIBRA_VERSION == 2
    // Set command regardless of which fluid system side is connected
    if (torque_dir == 0) {
        command = kNorthCmd;  // kAOut | kBOut
    } else {
        command = kSouthCmd;  // kAIn | kBIn
    }
#endif

    water_cmd_ = QByteArray(1, static_cast<char>(command));

    logger_->Debug("Water - Set command to " + BytesToStr(water_cmd_)
                   + " (A_IN | B_IN | A_OUT | B_OUT)");
}

/**
 * @brief Forces the state of a specific side of the fluid system.
 *
 * @param side The side to command
 * @param Water::State The state to force
 */
void ArduinoManager::ForceWaterCommand(const Water::Side& side,
                                       const Water::State& state) {
    if (!ser_water_->isOpen()) {
        logger_->Warn("Water - Cannot command Arduino: not connected!");
        return;
    }

    // Force-disable auto compensation
    SetAutoCompensation(false);

    uint8_t command = 0;

#if LIBRA_VERSION == 1
    // Set command based on specified side
    switch (side) {
        case Water::Side::kA:
            command = (state == Water::State::kFilling)    ? kAIn
                      : (state == Water::State::kDraining) ? kAOut
                                                           : 0;
            break;
        case Water::Side::kB:
            command = (state == Water::State::kFilling)    ? kBIn
                      : (state == Water::State::kDraining) ? kBOut
                                                           : 0;
            break;
        default:
            // Do nothing (since command is initialized at the top)
            break;
    }
#elif LIBRA_VERSION == 2
    // Set command regardless of which fluid system side is connected
    command = (state == Water::State::kFilling)    ? kSouthCmd
              : (state == Water::State::kDraining) ? kNorthCmd
                                                   : 0;
#endif

    water_cmd_ = QByteArray(1, static_cast<char>(command));

    logger_->Debug("Water - Forced command to " + BytesToStr(water_cmd_)
                   + " (A_IN | B_IN | A_OUT | B_OUT)");
}

//------------------------------------------------------------------------------
// !Manipulator Commands (slots)
//------------------------------------------------------------------------------

#if LIBRA_VERSION == 1
/**
 * @brief Attempts to establish a connection to the SerialServo Arduino.
 *
 * @param port_name The serial device address to connect to
 */
void ArduinoManager::ConnectManip(const QString& port_name) {
    // If there is already an active connection, gracefully terminate it
    DisconnectManip();

    // Set port options
    ser_manip_->setPortName(port_name);
    ser_manip_->setBaudRate(QSerialPort::Baud115200);
    ser_manip_->setDataBits(QSerialPort::Data8);
    ser_manip_->setParity(QSerialPort::NoParity);
    ser_manip_->setStopBits(QSerialPort::OneStop);
    ser_manip_->setFlowControl(QSerialPort::NoFlowControl);

    // Only continue if "open" was successful
    if (!ser_manip_->open(QIODevice::ReadWrite)) {
        emit ErrorThrown("Manip - Failed to open port: " + port_name + "!\n"
                         + ser_manip_->errorString());
        return;
    }

    logger_->Info("Manip - Connected to device at: " + port_name.toStdString());
    emit ManipConnected(true);

    // If update_timer_ is stopped, restart it
    RefreshUpdateTimerState();
}

/**
 * @brief Terminates the active connection.
 */
void ArduinoManager::DisconnectManip() {
    // This function is only called intentionally, so don't attempt to reconnect
    manip_needs_reconnect_ = false;

    if (ser_manip_->isOpen()) {
        ser_manip_->close();

        logger_->Debug("Manip - Gracefully disconnected from device");
        emit ManipConnected(false);
    }

    // If update_timer_ is running, and no other devices are connected, stop it
    RefreshUpdateTimerState();
}

/**
 * @brief Sets movement targets for all manipulator servos.
 *
 * @param arm_pitch Angle of LIBRA-I arm pitch joint "J3"
 * @param target_pan Target yaw angle
 * @param target_tilt Target pitch angle
 * @param move_slow Whether to use a slower, more controlled trajectory
 */
void ArduinoManager::SetManipCommand(const double& arm_pitch,
                                     const double& target_pan,
                                     const double& target_tilt,
                                     const bool& move_slow) {
    if (!ser_manip_->isOpen()) {
        logger_->Warn("Manip - Cannot command Arduino: not connected!");
        return;
    }

    m_current_pos_.at(kPitch) = arm_pitch;  // TODO: should update continuously
    m_target_pos_.at(kPan) = target_pan;
    m_target_pos_.at(kTilt) = target_tilt;

    if (move_slow) {
        // Only calculate the direction (UpdateManip takes care of position)
        auto get_direction = [](double target, double current) {
            return (target > current) ? 1 : (target < current) ? -1 : 0;
        };

        m_slow_direction_.at(kPan) = get_direction(target_pan,
                                                   m_current_pos_.at(kPan));
        m_slow_direction_.at(kTilt) = get_direction(target_tilt,
                                                    m_current_pos_.at(kTilt));
    } else {
        m_slow_direction_.at(kPan) = 0;
        m_slow_direction_.at(kTilt) = 0;

        // Setting the commanded positions to the target values causes the
        // servos to move at maximum speed (near-instant)
        m_current_pos_.at(kPan) = target_pan;
        m_current_pos_.at(kTilt) = target_tilt;
    }

    logger_->Debug("Manip - Set targets to " + std::to_string(target_pan) + " "
                   + std::to_string(target_tilt) + " deg");
}
#endif
