/******************************************************************************
 * @file   abstract_serial_device.cpp
 * @brief  Abstract control class for Arduino serial devices; implementation.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "abstract_serial_device.h"

// C++ Standard Library Headers
// (none)

// Other Library Headers
#include <QSerialPortInfo>  // Qt::SerialPort

// Project Headers
#include "qt_logger.h"

/* --- TABLE OF CONTENTS ---
 * !Class Management
 * !Device Commands (slots)
 * !Timer Management
 */

// Thread Timer Intervals

constexpr int kUpdateIntervalMs = 500;      // 0.5 sec (2 Hz)
constexpr int kReconnectIntervalMs = 3000;  // 3 sec (0.33 Hz)

//------------------------------------------------------------------------------
// !Class Management
//------------------------------------------------------------------------------

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param debug_mode Whether to output verbose debug text
 * @param device_name The logging identifier for this device
 */
AbstractSerialDevice::AbstractSerialDevice(QObject* parent, bool debug_mode,
                                           const std::string& device_name)
    : QObject(parent),
      debug_mode_(debug_mode),
      device_name_(device_name),
      serial_port_(new QSerialPort(this)) {
    // Initialize the logger
    logger_ = std::make_unique<QtLogger>(device_name_, debug_mode_);

    logger_->Debug("Starting " + device_name_ + "controller");

    // Configure timers
    update_timer_ = new QTimer(this);
    update_timer_->setInterval(kUpdateIntervalMs);

    reconnect_timer_ = new QTimer(this);
    reconnect_timer_->setInterval(kReconnectIntervalMs);

    // Connect timer signals (internal)
    connect(update_timer_, &QTimer::timeout, this,
            &AbstractSerialDevice::Update);
    connect(reconnect_timer_, &QTimer::timeout, this,
            &AbstractSerialDevice::TryReconnect);
}

/**
 * @brief Standard destructor.
 */
AbstractSerialDevice::~AbstractSerialDevice() {
    // Stop all timers
    update_timer_->stop();
    reconnect_timer_->stop();

    // Close the serial port connection
    if (serial_port_->isOpen()) {
        serial_port_->close();
    }

    logger_->Debug("Cleaned up " + device_name_ + "controller");
}

//------------------------------------------------------------------------------
// !Device Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Attempts to establish a connection to a serial device.
 *
 * @param port_name The serial device address to connect to
 */
void AbstractSerialDevice::Connect(const QString& port_name) {
    // If there is already an active connection, gracefully terminate it
    Disconnect();

    // Set port options
    serial_port_->setPortName(port_name);
    serial_port_->setBaudRate(QSerialPort::Baud115200);
    serial_port_->setDataBits(QSerialPort::Data8);
    serial_port_->setParity(QSerialPort::NoParity);
    serial_port_->setStopBits(QSerialPort::OneStop);
    serial_port_->setFlowControl(QSerialPort::NoFlowControl);

    // Only continue if "open" was successful
    if (!serial_port_->open(QIODevice::ReadWrite)) {
        emit ErrorThrown("[" + QString::fromStdString(device_name_)
                         + "]: Failed to open port " + port_name + "!\n"
                         + serial_port_->errorString());
        return;
    }

    logger_->Info("[" + device_name_ + "]: Connected to device at "
                  + port_name.toStdString());
    emit Connected(true);

    // Connection successful, resume operations
    reconnect_timer_->stop();
    update_timer_->start();
}

/**
 * @brief Terminates the active connection.
 */
void AbstractSerialDevice::Disconnect() {
    // This function is only called intentionally, so don't attempt to reconnect
    reconnect_timer_->stop();
    update_timer_->stop();

    if (serial_port_->isOpen()) {
        serial_port_->close();

        logger_->Debug("[" + device_name_
                       + "]: Gracefully disconnected from device");
        emit Connected(false);
    }
}

bool AbstractSerialDevice::IsOpen() const {
    return serial_port_->isOpen();
}

QString AbstractSerialDevice::PortName() const {
    return serial_port_->portName();
}

//------------------------------------------------------------------------------
// !Timer Management
//------------------------------------------------------------------------------

/**
 * @brief Executes device-specific update logic wrapped in reconnection handling.
 */
void AbstractSerialDevice::Update() {
    if (!IsOpen()) {
        return;
    }

    try {
        OnUpdate();
    } catch (const std::exception& e) {
        emit ErrorThrown("[" + QString::fromStdString(device_name_)
                         + "]: Communication error: " + QString(e.what()));

        // Close the problematic connection
        update_timer_->stop();
        serial_port_->close();
        emit Connected(false);

        // Periodically try to reconnect
        reconnect_timer_->start();
        logger_->Debug("[" + device_name_ + "]: Started reconnection timer");
    }
}

/**
 * @brief Attempts to reconnect to the device after a dropped connection.
 */
void AbstractSerialDevice::TryReconnect() {
    logger_->Debug("[" + device_name_
                   + "]: Attempting automatic reconnection...");
    Connect(serial_port_->portName());
}
