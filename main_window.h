/******************************************************************************
 * @file   main_window.h
 * @brief  Main app window header file.
 *
 * @author brice.c.aa
 * @date   2023/7/4
 ******************************************************************************/

// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   Qt
#include <QMainWindow>
// Project Headers
//   (none)

#pragma once

QT_BEGIN_NAMESPACE

namespace Ui {  // NOLINT: Qt-generated
class MainWindow;
}  // namespace Ui

QT_END_NAMESPACE

/**
 * @brief The main app window.
 */
class MainWindow : public QMainWindow {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // --- Main Window ---

    // --- Uncategorized ---

  private:
    // NOLINTEND
    // --- Helper Functions ---

    // --- Data Members ---
};
