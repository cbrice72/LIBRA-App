/******************************************************************************
 * @file   main_window.h
 * @brief  Main app window header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

// C++ Standard Library Headers
#include <fstream>

// Other Library Headers
#include <QMainWindow>  // Qt::Widgets
#include <QSerialPort>  // Qt::SerialPort
#include <QThread>      // Qt::Core

// Project Headers
#include "camera_manager.h"
#include "libra_hebi.h"
#include "lidar.h"  // unused

#pragma once

QT_BEGIN_NAMESPACE

namespace Ui {  // NOLINT: Qt-generated
class MainWindow;
}  // namespace Ui

QT_END_NAMESPACE

/**
 * @brief The primary control loop.
 */
class MainThread : public QThread {
    // NOLINTBEGIN: required by Qt
    Q_OBJECT
    // NOLINTEND

  public:
    MainThread(std::shared_ptr<LibraHebi> libra_arm,
               std::shared_ptr<QSerialPort> ser_water,
               std::shared_ptr<QSerialPort> ser_servo);
    ~MainThread();

    // --- Getters & Setters ---

    void SetDebugMode(bool enabled);

  public slots:
    void UpdateArmTarget(const std::array<double, 5>& input);

  private:
    void run() override;

    /**
     * @brief Logical status of the fluid system.
     */
    enum WaterMode { kStandby = 0, kAdjust, kDrain };

    // --- Helper Functions ---

    std::ofstream InitializeLog(std::string name);

    // --- Data Members ---

    bool debug_mode_{false};

    // LIBRA Components

    std::shared_ptr<LibraHebi> libra_arm_;
    std::shared_ptr<QSerialPort> ser_water_;
    std::shared_ptr<QSerialPort> ser_servo_;

    // Arm

    std::array<double, 5> arm_target_{0};
    std::array<std::array<double, 3>, 5> arm_current_{0};

    // Manipulator

    std::array<double, 3> manip_pos_{0};
    std::array<double, 3> manip_setpos_{0};
    std::array<int, 3> manip_dir_{0};

    // Pumps

    bool water_en_{true};
    WaterMode water_mode_{kStandby};

    // Logging

    std::ofstream continuous_log_;
    std::ofstream snapshot_log_;
};

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

  signals:
    void CommandArm(std::array<double, 5> target);

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);

    // EPOS Menu

    void on_a_epos_connect_triggered();
    void on_a_epos_disconnect_triggered();

    // HEBI Menu

    void on_a_hebi_connect_triggered();
    void on_a_hebi_disconnect_triggered();

    // Pumps Menu

    void on_a_pump_connect_triggered();
    void on_a_pump_disconnect_triggered();

    void on_a_pump_set_empty_triggered();
    void on_a_pump_set_full_triggered();

    // Manipulator Menu

    void on_a_manip_servos_connect_triggered();
    void on_a_manip_servos_disconnect_triggered();

    // LIDAR Menu

    void on_a_lidar_connect_triggered();
    void on_a_lidar_disconnect_triggered();

    void on_a_lidar_about_triggered();

    // --- Main Window ---

    // Arm

    void on_pb_arm_start_clicked();
    void on_pb_arm_stop_clicked();

    // Manipulator

    void on_pb_manip_slow_clicked();
    void on_pb_manip_fast_clicked();

    // Pumps

    void on_pb_pump_enable_clicked();
    void on_pb_pump_disable_clicked();
    void on_pb_pump_drain_clicked();

    // Camera

    void on_cb_camera_id_currentTextChanged(const QString& sel);
    void on_pb_camera_capture_clicked();
    void on_pb_camera_record_clicked();

    // Misc.

    void on_pb_logshot_clicked();

    // --- Uncategorized ---

  private:
    // NOLINTEND

    // --- Helper Functions ---

    void UpdatePumpVals();
    void UpdateServoVals();

    // --- Data Members ---

    Ui::MainWindow* ui_;

    bool debug_mode_{true};

    MainThread* main_thread_;  // primary control loop

    std::shared_ptr<LibraHebi> libra_arm_;
    std::shared_ptr<QSerialPort> ser_water_;
    std::shared_ptr<QSerialPort> ser_servo_;
    std::shared_ptr<Lidar> lidar_;  // unused

    std::unordered_map<QString, QString> available_cameras_;
    std::unique_ptr<CameraManager> camera_manager_;
};
