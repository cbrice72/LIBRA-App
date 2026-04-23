/******************************************************************************
 * @file   ros2_logger.h
 * @brief  ROS2 logging interface; header-only.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <iostream>

// Other Library Headers
#include <rclcpp/logger.hpp>  // ROS2 Core

// Project Headers
#include "logger.h"

/**
 * @brief Logger implementation that uses ROS2's logging API.
 *
 * @note Usage: `logger_ = std::make_unique<Ros2Logger>("my_parent", true,
 *                                                      this->get_logger())`.
 */
class Ros2Logger : public Logger {
  public:
    /**
     * @brief Implementation class constructor.
     *
     * @param parent_name Name of parent object
     * @param debug_mode Whether to initialize the logger level at DEBUG or INFO
     * @param logger Owning node's ROS2 logging object
     */
    explicit Ros2Logger(std::string parent_name, bool debug_mode,
                        rclcpp::Logger logger)
        : Logger(std::move(parent_name)), logger_(std::move(logger)) {
        SetDebugMode(debug_mode);
    }

    // --- Overrides ---

    void SetDebugMode(const bool& enabled) override {
        RCUTILS_LOG_SEVERITY level{RCUTILS_LOG_SEVERITY_INFO};  // default
        if (enabled) {
            level = RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG;
        }

        if (rcutils_logging_set_logger_level(logger_.get_name(), level)
            != RCUTILS_RET_OK) {
            std::cerr << "Failed to set logger level for " << logger_.get_name()
                      << std::endl;
        }
    }

    void Debug(const std::string& msg) override {
        const auto formatted = Format(msg);
        RCLCPP_DEBUG(logger_, "%s", formatted.c_str());
    }

    void Info(const std::string& msg) override {
        const auto formatted = Format(msg);
        RCLCPP_INFO(logger_, "%s", formatted.c_str());
    }

    void Warn(const std::string& msg) override {
        const auto formatted = Format(msg);
        RCLCPP_WARN(logger_, "%s", formatted.c_str());
    }

    void Error(const std::string& msg) override {
        const auto formatted = Format(msg);
        RCLCPP_ERROR(logger_, "%s", formatted.c_str());
    }

  private:
    rclcpp::Logger logger_;
};
