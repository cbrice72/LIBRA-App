/******************************************************************************
 * @file   diagnostics_widget.h
 * @brief  Runtime diagnostics display tabs; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstddef>
#include <optional>
#include <unordered_map>

// Other Library Headers
#include <QTimer>   // Qt::Core
#include <QWidget>  // Qt::Widgets
#ifdef BUILD_WITH_ROS2
# include <libra_interfaces/msg/cw_state.hpp>     // ROS2 Messages
# include <libra_interfaces/msg/diagnostics.hpp>  // ROS2 Messages
# include <rclcpp/rclcpp.hpp>                     // ROS2 Core
#endif

// Project Headers
#include "actuator_defs.h"
#ifdef BUILD_WITH_ROS2
# include "arduino_defs.h"
#endif

namespace Ui {  // NOLINT: Qt-generated
class DiagnosticsWidget;
}  // namespace Ui

/**
 * @brief Custom tab widget for displaying various runtime diagnostics.
 *        With ROS2 enabled, also publishes them for recording.
 */
class DiagnosticsWidget : public QWidget
#ifdef BUILD_WITH_ROS2
    ,
                          public rclcpp::Node
#endif
{
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit DiagnosticsWidget(QWidget* parent = nullptr);
    ~DiagnosticsWidget() override;

    // Delete copy constructor
    DiagnosticsWidget(const DiagnosticsWidget&) = delete;
    DiagnosticsWidget& operator=(const DiagnosticsWidget&) = delete;

  public slots:
    // --- Power ---

    void UpdateActuatorPower(const double& power, const Actuator::Type& type);
    void UpdateFluidPower(const double& power);

    // --- Torque ---

    void UpdatePitchTorque(const double& torque);
#if LIBRA_VERSION == 1
    void UpdateRollTorque(const double& torque);
#elif LIBRA_VERSION == 2
    void UpdateModelTorque(const std::optional<double>& torque);
#endif

    void SetTorqueLimits(const double& lower, const double& upper);
    void ClearTorques();

#ifdef BUILD_WITH_ROS2
    // --- CW (ROS2 only) ---

    void UpdateCwTarget(const std::optional<double>& volume);
    void UpdateCwState(const Water::Side& side, const double& volume,
                       const double& flow);
#endif

  private slots:
    // --- Power ---

    void on_pb_energy_clear_clicked();

  private:
    // --- Helper Functions ---

    void SamplePower();
    void SampleTorque();
#ifdef BUILD_WITH_ROS2
    void PublishLatest();
#endif

    // --- Data Members ---

    Ui::DiagnosticsWidget* ui_{nullptr};

    // Power

    std::unordered_map<Actuator::Type, double> actuator_power_;  // latest (W)
    double fluid_power_{0.0};                                    // latest (W)

    double energy_{0.0};  // total since start or last clear (Wh)

    std::size_t actuators_series_{0};
    std::size_t fluid_series_{0};

    // Torque (joint space)

    std::optional<double> pitch_torque_;  // latest measured (Nm)
#if LIBRA_VERSION == 1
    std::optional<double> roll_torque_;  // latest measured (Nm)
#elif LIBRA_VERSION == 2
    std::optional<double> model_torque_;  // latest predicted (Nm)
#endif

    std::size_t torque_meas_series_{0};
#if LIBRA_VERSION == 2
    std::size_t torque_model_series_{0};
#endif

    QTimer* sample_timer_{nullptr};

#ifdef BUILD_WITH_ROS2
    // ROS2

    rclcpp::Publisher<libra_interfaces::msg::Diagnostics>::SharedPtr
        diagnostics_pub_;
    rclcpp::Publisher<libra_interfaces::msg::CwState>::SharedPtr cw_state_pub_;
    libra_interfaces::msg::Diagnostics diagnostics_msg_;
    libra_interfaces::msg::CwState cw_state_msg_;

    QTimer* publish_timer_{nullptr};
#endif
};
