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
 * @brief Abstract interface for class-level logging.
 *
 * @see QtLogger
 * @see Ros2Logger
 */
class Logger {
  public:
    virtual ~Logger() = default;

    /**
     * @brief Whether to output debug-level messages.
     *
     * @param enabled If debug-level messages should be enabled.
     *
     * @note Any info-, warning-, and error-level messages are always output.
     */
    virtual void SetDebugMode(const bool& enabled) = 0;

    /**
     * @brief Log a debug-level message.
     *
     * @param msg The message to log
     */
    virtual void Debug(const std::string& msg) = 0;

    /**
     * @brief Log an info-level message.
     *
     * @param msg The message to log
     */
    virtual void Info(const std::string& msg) = 0;

    /**
     * @brief Log a warning-level message.
     *
     * @param msg The message to log
     */
    virtual void Warn(const std::string& msg) = 0;

    /**
     * @brief Log an error-level message.
     *
     * @param msg The message to log
     */
    virtual void Error(const std::string& msg) = 0;
};
