/******************************************************************************
 * @file   abstract_serial_device.h
 * @brief  Abstract control class for Arduino serial devices; header-only.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <memory>
#include <string>

// Other Library Headers
#include <QObject>      // Qt::Core
#include <QSerialPort>  // Qt::SerialPort
#include <QTimer>       // Qt::Core

// Project Headers
#include "logger.h"

/**
 * @brief Provides a common interface for a heterogeneous mix of serial devices.
 *        Handles its own background update and reconnection loops.
 *
 * @note If you are unfamiliar with abstract classes, they essentially just
 *       provide an interface for making multiple of types of similar objects.
 *       For example: `Fruit` can be used to define `Apple`, `Banana`, and
 *       `Orange`. Abstract classes contain common functions (defined normally)
 *       and pure virtual functions, denoted by the `= 0` at the end. This tells
 *       the compiler that it shouldn't allow a derivative class to compile if
 *       it doesn't first define those pure virtual functions (using `override`).
 */
class AbstractSerialDevice : public QObject {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    AbstractSerialDevice() = delete;
    explicit AbstractSerialDevice(QObject* parent, bool debug_mode,
                                  const std::string& device_name);
    ~AbstractSerialDevice() override;

    bool IsOpen() const;
    QString PortName() const;

  public slots:

    void SetDebugMode(const bool& enabled) {
        debug_mode_ = enabled;
        logger_->SetDebugMode(debug_mode_);
    };

    // --- Device Commands ---

    void Connect(const QString& port_name);
    void Disconnect();

  signals:
    // --- Device Updates ---

    void Connected(const bool& connected);
    void ErrorThrown(const QString& err);

  protected:
    virtual void OnUpdate() = 0;

    // --- Data Members (per-device) ---

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{false};

    const std::string device_name_;
    QSerialPort* serial_port_{nullptr};

  private slots:
    // --- Timer Management ---

    void Update();
    void TryReconnect();

  private:
    // --- Data Members (common) ---

    QTimer* update_timer_{nullptr};
    QTimer* reconnect_timer_{nullptr};
};
