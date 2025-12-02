/******************************************************************************
 * @file   torque_comp_dialog.h
 * @brief  Torque compensation settings dialog box header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDialog>  // Qt::Widgets

// Project Headers
//   (none)

namespace Ui {  // NOLINT: Qt-generated
class TorqueCompDialog;
}  // namespace Ui

/**
 * @brief A dialog for configuring torque compensation bounds for HEBI actuators.
 */
class TorqueCompDialog : public QDialog {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit TorqueCompDialog(QWidget* parent = nullptr,
                              const double& lower_bound = 2.5,
                              const double& upper_bound = 5.0);
    ~TorqueCompDialog();

    double GetLowerBound() const;
    double GetUpperBound() const;

    // NOLINTBEGIN: Qt-generated
  private slots:

  private:
    // NOLINTEND
    // --- Data Members ---

    Ui::TorqueCompDialog* ui_{nullptr};
};