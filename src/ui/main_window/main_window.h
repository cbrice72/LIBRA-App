/******************************************************************************
 * @file   main_window.h
 * @brief  Main app window header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <fstream>

// Other Library Headers
#include <QLabel>       // Qt::Widgets
#include <QMainWindow>  // Qt::Widgets

// Project Headers
// - Actuators
#include "hebi_thread.h"
#if LIBRA_VERSION == 2
# include "epos_thread.h"
#endif
// - Device Managers
#include "camera_manager.h"
#include "flow_controller.h"
#include "water_controller.h"
#if LIBRA_VERSION == 1
# include "manip_controller.h"
#endif
// - Utilities
#include "logger.h"

QT_BEGIN_NAMESPACE

namespace Ui {  // NOLINT: Qt-generated
class MainWindow;
}  // namespace Ui

QT_END_NAMESPACE

// Type alias for conveniently accessing a feedback label, where rows are
// joints and columns are feedback types
using FeedbackElementMapOfMaps = std::unordered_map<
    Joint::Name, std::unordered_map<Actuator::Feedback, QLabel*>>;

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
    void HandleCriticalError(const QString& err);

    // --- Arduino Updates ---

    void HandleWaterConnChanged(const bool& connected);
    void HandleWaterStatus(const Water::Side& side, const Water::State& state);

    void HandleFlowConnChanged(const bool& connected);
    void HandleFlowStatus(const double& inflow, const double& outflow);

#if LIBRA_VERSION == 1
    void HandleManipConnChanged(const bool& connected);
    void HandleManipPosition(const double& base, const double& pan,
                             const double& tilt);
#endif

    // --- Actuator Updates ---

    void HandleHebiConnChanged(const bool& connected);
#if LIBRA_VERSION == 2
    void HandleEposConnChanged(const bool& connected);
#endif

    void HandleActuatorFeedback(
        const std::unordered_map<Joint::Name, double>& feedbacks,
        const Actuator::Feedback feedback_type);
    void HandleActuatorStatus(const QString& status, const Actuator::Type type);

  signals:
    // --- Generic Commands ---

    void EnableDebugMode(const bool& enabled);

    void EnableAutoTorqueComp(const bool& enabled);

    // --- Arduino Commands ---

    void ConnectWater(const QString& port_name);
    void CommandWater(const Water::Side& side, const Water::State& state);

    void ConnectFlow(const QString& port_name);

#if LIBRA_VERSION == 1
    void ConnectManip(const QString& port_name);
    void CommandManip(const double& pan, const double& tilt,
                      const bool& move_slow);
#endif

    // --- Actuator Commands ---

    void CommandHebi(const std::vector<double>& deg);
    void LoadHebiGains(const QString& file_path);
    void UpdateHebiTorqueCompBounds(const double& lower_bound,
                                    const double& upper_bound);

#if LIBRA_VERSION == 2
    void ConnectEpos();
    void CommandEpos(const std::vector<double>& deg);
#endif

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);

    // Automation Menu

    void on_a_atc_update_bounds_triggered();
    void on_a_atc_ignore_water_level_triggered(bool checked);

    // Actuators Menu

#if LIBRA_VERSION == 2
    void on_a_epos_connect_triggered();
#endif

    void on_a_hebi_load_gains_triggered();
    void on_a_hebi_override_limits_toggled(bool checked);

#if LIBRA_VERSION == 1
    void on_a_manip_connect_triggered();
#endif

    // Sensors Menu
    // (most actions are handled via signals, and thus don't need functions)

    void on_a_refresh_camera_list_triggered();

    // Water Menu

    void on_a_water_cw_connect_triggered();
    void on_a_water_flow_connect_triggered();

    void on_a_water_set_empty_triggered();
    void on_a_water_set_full_triggered();

    // Quick Actions Menu

    void on_a_connect_all_triggered();
    void on_a_disconnect_all_triggered();

    // --- Main Window ---

    // Arm

    void on_pb_quick_input_clicked();

    void on_pb_arm_start_clicked();
    // (pb_arm_stop is handled via signals)
    void on_pb_atc_enable_toggled(bool checked);

    // Water

    void on_pb_water_fill_A_clicked(bool checked);
    void on_pb_water_drain_A_clicked(bool checked);
#if LIBRA_VERSION == 1
    void on_pb_water_fill_B_clicked(bool checked);
    void on_pb_water_drain_B_clicked(bool checked);
#endif

    // Manipulator

#if LIBRA_VERSION == 1
    void on_pb_manip_slow_clicked();
    void on_pb_manip_fast_clicked();
#endif

    // Camera

    void on_cb_camera_name_currentTextChanged(const QString& sel);
    void on_pb_camera_capture_clicked();
    void on_pb_camera_record_clicked();
    void on_pb_camera_flip_h_clicked();
    void on_pb_camera_flip_v_clicked();

    // Misc.

    // --- Uncategorized ---

  private:
    // NOLINTEND

    // --- Helper Functions ---

    void ConfigureUi();
    void InitializeFeedbackElementMap();
    void InitializeDeviceManagers();
    void InitializeThreads();

    void UpdateActuatorControls();
    void UpdateATCControls();

    // --- Data Members ---

    Ui::MainWindow* ui_{nullptr};

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{true};

    FeedbackElementMapOfMaps feedback_element_map_;

    std::vector<double> last_command_;  // actuator-space target positions

    bool ignore_water_level_{false};

    // Device Managers

    WaterController* water_controller_{nullptr};  // water Arduino control
    FlowController* flow_controller_{nullptr};    // flow Arduino control
#if LIBRA_VERSION == 1
    ManipController* manip_controller_{nullptr};  // manipulator Arduino control
#endif
    std::unique_ptr<CameraManager> camera_manager_;  // media device control

    std::unordered_map<QString, QString> available_cameras_;  // name, path

    // Threads

    HebiThread* hebi_thread_{nullptr};  // HEBI actuator control
#if LIBRA_VERSION == 2
    EposThread* epos_thread_{nullptr};  // EPOS (Maxon) actuator control
#endif

    // Temporary Value Holders

    struct ArmLimits {
        double pitch_min{-180.0};
        double pitch_max{180.0};
#if LIBRA_VERSION != 1
        double yaw_min{-180.0};
        double yaw_max{180.0};
#elif LIBRA_VERSION != 2
        double roll_min{-180.0};
        double roll_max{180.0};
        double j1_min{-180.0};
        double j1_max{180.0};
        double j2_min{-180.0};
        double j2_max{180.0};
        double j3_min{-180.0};
        double j3_max{180.0};
#endif
    } original_limits_;
};
