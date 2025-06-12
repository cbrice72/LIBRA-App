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
#include <QLabel>       // Qt::Widgets
#include <QMainWindow>  // Qt::Widgets

// Project Headers
#include "arduino_thread.h"
#include "camera_manager.h"
#include "hebi_thread.h"
#include "log_thread.h"
#if LIBRA_VERSION == 2
# include "epos_thread.h"
#endif

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

    // --- Arduino Updates ---

    void HandleManipConnChanged(const bool& enabled);
    void HandlePumpConnChanged(const bool& enabled);

    // --- Actuator Updates ---

    void HandleActuatorFeedback(
        const std::unordered_map<Actuator::Joint, double>& feedbacks,
        const Actuator::Feedback feedback_type);
    void HandleActuatorStatus(const QString& status, const Actuator::Type type);

  signals:
    // --- Generic Commands ---

    void UpdateDebugMode(const bool& enabled);

    // --- Arduino Commands ---

    void CommandManip(const double& arm_pitch, const double& pan,
                      const double& tilt, const bool& move_slow);
    void CommandPump(const PumpState& state);

    // --- Actuator Commands ---

    void CommandHebi(const std::vector<double>& deg);
#if LIBRA_VERSION == 2
    void CommandEpos(const std::vector<double>& deg);
#endif

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);

    // Actuators Menu

#if LIBRA_VERSION == 2
    void on_a_epos_connect_triggered();
    void on_a_epos_disconnect_triggered();
#endif

    void on_a_hebi_connect_triggered();
    void on_a_hebi_disconnect_triggered();

    // Sensors Menu
    // (some actions are handed off to signals, and thus don't need functions)

    void on_a_refresh_camera_list_triggered();

    void on_a_lidar_connect_triggered();
    void on_a_lidar_disconnect_triggered();
    void on_a_lidar_about_triggered();

    // Pump Menu

    void on_a_pump_set_empty_triggered();
    void on_a_pump_set_full_triggered();

    // Quick Actions Menu

    void on_a_connect_all_triggered();
    void on_a_disconnect_all_triggered();

    // --- Main Window ---

    // Arm

    void on_pb_arm_start_clicked();

    // Manipulator

    void on_pb_manip_slow_clicked();
    void on_pb_manip_fast_clicked();

    // Pump

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

    void ConfigureUi();
    void InitializeThreads();
    void InitializeFeedbackElementMap();

    void UpdateManipVals();
    void UpdatePumpVals();

    // --- Data Members ---

    Ui::MainWindow* ui_;

    bool debug_mode_{true};

    FeedbackElementMapOfMaps feedback_element_map_;

    LogThread* log_thread_{nullptr};          // consolidated logging
    ArduinoThread* arduino_thread_{nullptr};  // serial device control
    HebiThread* hebi_thread_{nullptr};        // HEBI actuator control
#if LIBRA_VERSION == 2
    EposThread* epos_thread_{nullptr};  // EPOS (Maxon) actuator control
#endif

    std::unordered_map<QString, QString> available_cameras_;  // ID, desc
    std::unique_ptr<CameraManager> camera_manager_;
};
