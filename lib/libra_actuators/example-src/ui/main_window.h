/******************************************************************************
 * @file   main_window.h
 * @brief  Main app window header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QMainWindow>  // Qt::Widgets

// Project Headers
#include "arm_thread.h"

QT_BEGIN_NAMESPACE

namespace Ui {  // NOLINT: Qt-generated
class MainWindow;
}  // namespace Ui

QT_END_NAMESPACE

/**
 * @brief The main command app window.
 */
class MainWindow : public QMainWindow {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

  public slots:
    void HandleStatusMsg(const QString& status);
    void HandleErrorMsg(const QString& err);

  signals:
    void TryConnect(const Actuator::Joint& joint);
    void TryDisconnect(const Actuator::Joint& joint);

    void CommandOne(const Actuator::Joint& joint, const double& val);
    void CommandAll(const std::vector<double>& vals);

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);

    // EPOS menu

    void on_a_epos_connect_triggered();
    void on_a_epos_disconnect_triggered();

    // HEBI menu

    void on_a_hebi_connect_triggered();
    void on_a_hebi_disconnect_triggered();

    // --- Main Window ---

    void on_pb_yaw_start_clicked();
    void on_pb_pitch_start_clicked();

    void on_pb_arm_start_clicked();

    // --- Uncategorized ---

  private:
    // NOLINTEND

    // --- Helper Functions ---

    // --- Data Members ---

    Ui::MainWindow* ui_;
    ArmThread* arm_thread_;

    bool debug_mode_{false};
};
