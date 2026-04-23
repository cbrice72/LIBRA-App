/******************************************************************************
 * @file   logger.h
 * @brief  Common logging interface (backend-agnostic); header-only.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Other Library Headers
//   (none)

// Project Headers
//   (none)

/**
 * @brief Abstract interface for class-level logging. Allows you to use the same
 *        functions to output messages regardless of the logging backend.
 *
 * @note Usage:
 *         1) In your class declaration, declare a `Logger` member variable.
 *         2) In your class constructor, initialize the member as one of the
 *            classes which derive from `Logger` (e.g., `QtLogger`).
 *
 * @see QtLogger Ros2Logger
 */
class Logger {
  public:
    explicit Logger(std::string parent_name)
        : parent_prefix_("[" + std::move(parent_name) + "] ") {}

    virtual ~Logger() = default;

    /**
     * @brief Updates the log message level.
     *
     * @param enabled Whether to enable debug-level messages.
     *
     * @note Any info-, warning-, and error-level messages are always output.
     */
    virtual void SetDebugMode(const bool& enabled) = 0;

    /**
     * @brief Logs a debug-level message.
     *
     * @param msg The message to log
     */
    virtual void Debug(const std::string& msg) = 0;

    /**
     * @brief Logs an info-level message.
     *
     * @param msg The message to log
     */
    virtual void Info(const std::string& msg) = 0;

    /**
     * @brief Logs a warning-level message.
     *
     * @param msg The message to log
     */
    virtual void Warn(const std::string& msg) = 0;

    /**
     * @brief Logs an error-level message.
     *
     * @param msg The message to log
     */
    virtual void Error(const std::string& msg) = 0;

  protected:
    [[nodiscard]] std::string Format(const std::string& msg) const {
        return parent_prefix_ + msg;
    }

  private:
    std::string parent_prefix_;
};
