/******************************************************************************
 * @file   reactive_cw_dialog.h
 * @brief  Reactive counterweight settings dialog box; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDialog>  // Qt::Widgets

// Project Headers
//   (none)

namespace Ui {  // NOLINT: Qt-generated
class ReactiveCwDialog;
}  // namespace Ui

/**
 * @brief A dialog for configuring reactive counterweight torque bounds.
 */
class ReactiveCwDialog : public QDialog {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit ReactiveCwDialog(QWidget* parent = nullptr,
                              const double& lower_bound = 2.5,
                              const double& upper_bound = 5.0);
    ~ReactiveCwDialog() override;

    double GetLowerBound() const;
    double GetUpperBound() const;

    // NOLINTBEGIN: Qt-generated
  private slots:

  private:
    // NOLINTEND
    // --- Data Members ---

    Ui::ReactiveCwDialog* ui_{nullptr};
};