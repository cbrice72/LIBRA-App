/******************************************************************************
 * @file   main_window.h
 * @brief  Main app window header file.
 *
 * @author brice.c.aa
 * @date   2024/2/22
 ******************************************************************************/

// C++ Standard Library Headers
#include <fstream>
// Other Libraries' Headers
//   Qt
#include <QMainWindow>
#include <QThread>
// Project Headers
#include "libra_hebi.h"
#include "serial.h"

#pragma once

QT_BEGIN_NAMESPACE

namespace Ui {  // NOLINT: Qt-generated
class MainWindow;
}  // namespace Ui

QT_END_NAMESPACE

/**
 * @brief TODO(brice.c.aa)
 *
 * @todo is this necessary in the new app?
 */
/*
class MainThread : public QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    MainThread() = default;
    ~MainThread() = default;

  private:
    void run() override;
};
*/

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

    // Input

    void on_pb_arm_start_clicked();
    void on_pb_arm_stop_clicked();
    void on_pb_arm_convert_clicked();

    // Pumps

    void on_pb_pumps_enable_clicked();
    void on_pb_pumps_disable_clicked();
    void on_pb_pumps_drain_clicked();

    // Camera

    void on_pb_camera_slow_clicked();
    void on_pb_camera_fast_clicked();

    // Misc.

    void on_pb_logshot_clicked();

    // --- Uncategorized ---

  private:
    // NOLINTEND

    /**
     * @brief Logical status of the fluid system.
     */
    enum WaterMode { kStandby = 0, kAdjust, kDrain };

    // --- Helper Functions ---

    void PrintDeviceList();
    void ConnectSerial();
    void ConnectHebi();

    // --- Data Members ---
    Ui::MainWindow* ui_;

    // TODO: is this necessary in the new app?
    // MainThread* main_thread_;  // primary loop

    std::unique_ptr<LibraHebi> libra_arm_;
    std::unique_ptr<Serial> ser_water_;
    std::unique_ptr<Serial> ser_servo_;

    // TODO(brice.c.aa): possibly unneeded in Qt implementation
    volatile bool flag_thread_end_{0};
    volatile bool flag_end_{0};

    std::ofstream continuous_log_;
    std::ofstream snapshot_log_;

    bool water_en_{true};
    WaterMode water_mode_{kStandby};

    std::array<double, 5> input_{0};
    std::array<std::array<double, 3>, 5> value_{0};
    std::array<double, 3> camera_pos_{0};
    std::array<double, 3> camera_setpos_{0};
    std::array<int, 3> camera_dir_{0};
};
