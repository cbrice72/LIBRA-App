/******************************************************************************
 * @file   diagnostics_widget.h
 * @brief  Runtime diagnostics display tabs; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <cstddef>
#include <unordered_map>

// Other Library Headers
#include <QTimer>   // Qt::Core
#include <QWidget>  // Qt::Widgets

// Project Headers
#include "actuator_defs.h"

namespace Ui {  // NOLINT: Qt-generated
class DiagnosticsWidget;
}  // namespace Ui

/**
 * @brief Custom tab widget for displaying various runtime diagnostics.
 */
class DiagnosticsWidget : public QWidget {
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

    // NOLINTBEGIN: Qt-generated
  private slots:
    void on_cb_power_window_currentIndexChanged(int index);

  private:
    // NOLINTEND
    // --- Helper Functions ---

    void SamplePower();

    // --- Data Members ---

    Ui::DiagnosticsWidget* ui_{nullptr};

    // Power

    std::unordered_map<Actuator::Type, double> actuator_power_;  // latest (W)
    double fluid_power_{0.0};                                    // latest (W)

    std::size_t actuators_series_{0};
    std::size_t fluid_series_{0};

    QTimer* power_timer_{nullptr};  // samples latest values for graph/table
};
