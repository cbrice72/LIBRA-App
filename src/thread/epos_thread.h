/******************************************************************************
 * @file   epos_thread.h
 * @brief  Control class for LIBRA EPOS (Maxon) actuators; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <mutex>
#include <string>

// Other Library Headers
#ifdef BUILD_WITH_ROS2
# include <rclcpp/rclcpp.hpp>  // ROS2 Core
#endif

// Project Headers
#include "abstract_actuator_thread.h"

/**
 * @brief Control class for EPOS (Maxon) actuators.
 *
 * @note Since I don't see a reason to use `EposThread` outside of the LIBRA
 *       project in the near future, and I only need at most one EPOS actuator,
 *       I won't go through the trouble of making full use of the EPOS library
 *       to match the HEBI API's one-group-to-many-actuators functionality.
 *
 * @see abstract_actuator_thread
 */
class EposThread : public AbstractActuatorThread
#ifdef BUILD_WITH_ROS2
    ,
                   public rclcpp::Node
#endif
{
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit EposThread(QObject* parent, std::string device_name,
                        std::string protocol_name, std::string interface_name,
                        std::string port_name, uint baud_rate,
                        const bool& debug_mode = false);
    ~EposThread() override;

  public slots:
    // --- Actuator Commands ---

    void Connect() override;
    void Disconnect() override;

    void SetTarget(const std::vector<double>& target) override;
    void Stop() override;

  signals:
    // --- Actuator Updates ---

    // NOTE: see AbstractActuatorThread for generic signals

  private:
    void run() override;

    // --- Helper Functions ---

    QString GetStatus();

    // --- Data Members ---

    std::string device_name_;
    std::string protocol_name_;
    std::string interface_name_;
    std::string port_name_;
    uint baud_rate_;

    void* handle_{nullptr};  // void* are dangerous, but Maxon handles use them
    std::mutex handle_mutex_;

    int target_;  // in inc, not deg
    int last_target_;
};
