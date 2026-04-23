/******************************************************************************
 * @file   qt_logger.h
 * @brief  Qt logging interface (backend-agnostic); header-only.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
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
        : Logger(std::move(parent_name)), debug_mode_(debug_mode) {}

    // --- Overrides ---

    void SetDebugMode(const bool& enabled) override {
        debug_mode_ = enabled;
    }

    void Debug(const std::string& msg) override {
        if (debug_mode_) {
            qDebug().noquote() << QString::fromStdString(Format(msg));
        }
    }

    void Info(const std::string& msg) override {
        qInfo().noquote() << QString::fromStdString(Format(msg));
    }

    void Warn(const std::string& msg) override {
        qWarning().noquote() << QString::fromStdString(Format(msg));
    }

    void Error(const std::string& msg) override {
        qCritical().noquote() << QString::fromStdString(Format(msg));
    }

  private:
    bool debug_mode_;
};
