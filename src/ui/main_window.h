/******************************************************************************
 * @file   main_window.h
 * @brief  Main app window header file.
 *
 * @author brice.c.aa
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <fstream>

// Other Library Headers
#include <QMainWindow>  // Qt::Widgets
#include <QSerialPort>  // Qt::SerialPort

// Project Headers
#include "camera_manager.h"
#include "epos_thread.h"
#include "hebi_thread.h"
#include "log_thread.h"

QT_BEGIN_NAMESPACE

namespace Ui {  // NOLINT: Qt-generated
class MainWindow;
}  // namespace Ui

QT_END_NAMESPACE

// Type alias for conveniently accessing a feedback label, where rows are
// actuators and columns are feedback types (order follows MainWindow UI)
using FeedbackElementMapOfMaps = std::unordered_map<
    Actuator::Joint, std::unordered_map<Actuator::Feedback, QLabel*>>;

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
    void HandleErrorMsg(const QString& err);

    // --- Actuator Updates ---

    void HandleActuatorFeedback(
        const std::unordered_map<Actuator::Joint, double>& feedbacks,
        const Actuator::Feedback feedback_type);
    void HandleActuatorStatus(const QString& status, const Actuator::Type type);

  signals:
    void UpdateDebugMode(const bool& enabled);

    // --- Actuator Commands ---

    void CommandEpos(const std::vector<double>& deg);
    void CommandHebi(const std::vector<double>& deg);

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);

    // EPOS Menu

    // void on_a_epos_update_triggered();  // TODO

    // HEBI Menu

    // void on_a_hebi_update_triggered();  // TODO

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

    // --- Uncategorized ---

  private:
    // NOLINTEND

    // --- Helper Functions ---

    void InitializeFeedbackElementMap();

    void UpdatePumpVals();
    void UpdateServoVals();

    // --- Data Members ---

    Ui::MainWindow* ui_;

    bool debug_mode_{true};

    FeedbackElementMapOfMaps feedback_element_map_;

    LogThread* log_thread_{nullptr};    // consolidated logging
    EposThread* epos_thread_{nullptr};  // EPOS (Maxon) actuator control
    HebiThread* hebi_thread_{nullptr};  // HEBI actuator control

    QSerialPort* ser_water_{nullptr};
    QSerialPort* ser_servo_{nullptr};

    std::unordered_map<QString, QString> available_cameras_;
    std::unique_ptr<CameraManager> camera_manager_;
};
