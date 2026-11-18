/******************************************************************************
 * @file   diagnostics_widget.h
 * @brief  Runtime diagnostics display tabs; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QWidget>  // Qt::Widgets

// Project Headers
//   (none)

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

    // NOLINTBEGIN: Qt-generated
  private slots:
    void on_cb_power_window_currentIndexChanged(int index);

  private:
    // NOLINTEND
    // --- Data Members ---

    Ui::DiagnosticsWidget* ui_{nullptr};
};
