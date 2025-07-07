/******************************************************************************
 * @file   qt_logger.h
 * @brief  Qt logging interface (backend-agnostic); header-only.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDebug>  // Qt::Core

// Project Headers
#include "logger.h"

/**
 * @brief Logger implementation that uses Qt's logging API.
 *
 * @note Usage: `logger_ = std::make_unique<QtLogger>()`.
 */
class QtLogger : public Logger {
  public:
    /**
     * @brief Implementation constructor.
     *
     * @param debug_mode Whether to enable DEBUG messages at start
     */
    explicit QtLogger(bool debug_mode) : debug_mode_(debug_mode) {}

    // --- Overrides ---

    void SetDebugMode(const bool& enabled) override {
        debug_mode_ = enabled;
    }

    void Debug(const std::string& msg) override {
        if (debug_mode_) {
            qDebug().noquote() << "[DEBUG]" << QString::fromStdString(msg);
        }
    }

    void Info(const std::string& msg) override {
        qInfo().noquote() << "[INFO]" << QString::fromStdString(msg);
    }

    void Warn(const std::string& msg) override {
        qWarning().noquote() << "[WARN]" << QString::fromStdString(msg);
    }

    void Error(const std::string& msg) override {
        qCritical().noquote() << "[ERROR]" << QString::fromStdString(msg);
    }

  private:
    bool debug_mode_;
};
