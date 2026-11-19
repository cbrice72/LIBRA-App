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

#ifdef BUILD_WITH_ROS2
    // --- Torque and CW (ROS2 only) ---

    void UpdateModelDiagnostics(const double& torque_meas,
                                const std::optional<double>& torque_ext_g,
                                const std::optional<double>& cw_volume_tgt);
    void UpdateCwState(const Water::Side& side, const double& volume,
                       const double& flow);
#endif

    // NOLINTBEGIN: Qt-generated
  private slots:
    void on_cb_power_window_currentIndexChanged(int index);

  private:
    // NOLINTEND
    // --- Helper Functions ---

    void SamplePower();
#ifdef BUILD_WITH_ROS2
    void PublishLatest();
#endif

    // --- Data Members ---

    Ui::DiagnosticsWidget* ui_{nullptr};

    // Power

    std::unordered_map<Actuator::Type, double> actuator_power_;  // latest (W)
    double fluid_power_{0.0};                                    // latest (W)

    std::size_t actuators_series_{0};
    std::size_t fluid_series_{0};

    QTimer* power_timer_{nullptr};  // samples latest values for graph/table

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
