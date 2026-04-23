/******************************************************************************
 * @file   qt_logger.h
 * @brief  Qt logging interface (backend-agnostic); header-only.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <chrono>
#include <iomanip>
#include <ostream>
#include <utility>

// Other Library Headers
#include <QDebug>  // Qt::Core

// Project Headers
#include "logger.h"

/**
 * @brief Logger implementation that uses Qt's logging API.
 *
 * @note Usage: `logger_ = std::make_unique<QtLogger>("my_parent", true)`.
 */
class QtLogger : public Logger {
  public:
    /**
     * @brief Implementation constructor.
     *
     * @param parent_name Name of parent object
     * @param debug_mode Whether to enable DEBUG messages at start
     */
    explicit QtLogger(std::string parent_name, bool debug_mode)
        : parent_name_(std::move(parent_name)), debug_mode_(debug_mode) {}

    // --- Overrides ---

    void SetDebugMode(const bool& enabled) override {
        debug_mode_ = enabled;
    }

    void Debug(const std::string& msg) override {
        if (debug_mode_) {
            qDebug().noquote() << QString::fromStdString(Format("DEBUG", msg));
        }
    }

    void Info(const std::string& msg) override {
        qInfo().noquote() << QString::fromStdString(Format("INFO", msg));
    }

    void Warn(const std::string& msg) override {
        qWarning().noquote() << QString::fromStdString(Format("WARN", msg));
    }

    void Error(const std::string& msg) override {
        qCritical().noquote() << QString::fromStdString(Format("ERROR", msg));
    }

  private:
    /**
     * @brief Returns a ROS2-like formatted log message.
     *
     * @param level Log level
     * @param msg Log message
     */
    [[nodiscard]] std::string Format(const std::string& level,
                                     const std::string& msg) {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now)
                         .count()
                     % 1'000'000'000LL;

        std::ostringstream oss;
        oss << "[" << level << "] "
            << "[" << secs << "." << std::setfill('0') << std::setw(9) << nanos
            << "] "
            << "[" << parent_name_ << "]: " << msg;

        return oss.str();
    }

    std::string parent_name_;
    bool debug_mode_;
};
