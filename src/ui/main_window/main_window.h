/******************************************************************************
 * @file   main_window.h
 * @brief  Main app window; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <fstream>
#include <memory>
#include <optional>
#include <unordered_map>

// Other Library Headers
#include <QDoubleSpinBox>  // Qt::Widgets
#include <QLabel>          // Qt::Widgets
#include <QMainWindow>     // Qt::Widgets
#include <QTimer>          // Qt::Core

// Project Headers
// - Actuators
#include "hebi_thread.h"
#if LIBRA_VERSION == 2
# include "epos_thread.h"
# include "libra2_motion_controller.h"
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

    // --- Arduino Updates (main_window.cpp) ---

    void HandleWaterConnChanged(const bool& connected);
    void HandleWaterStatus(const Water::Side& side, const Water::State& state);
    void HandleWaterVolume(const Water::Side& side, const double& volume);
    void HandleWaterTankLimit(const Water::Side& side, const bool& full);

    void HandleFlowConnChanged(const bool& connected);
    void HandleFlowStatus(const double& flow);

#if LIBRA_VERSION == 1
    void HandleManipConnChanged(const bool& connected);
    void HandleManipPosition(const double& base, const double& pan,
                             const double& tilt);
#endif

    // --- Actuator Updates (main_window.cpp) ---

    void HandleHebiConnChanged(const bool& connected);
#if LIBRA_VERSION == 2
    void HandleEposConnChanged(const bool& connected);
#endif

    void HandleActuatorFeedback(
        const std::unordered_map<Joint::Name, double>& feedbacks,
        const Actuator::Feedback& feedback_type);
    void HandleActuatorTelemetry(const QString& telemetry,
                                 const Actuator::Type& type);
    void HandleMovementStateChanged();

#if LIBRA_VERSION == 2
    void HandlePitchReleaseChanged(const bool& released);

    void HandleCalibrationChanged(const Motion::Calibration& level);
    void HandleCalibrationProgress(const QString& text, const bool& running);
#endif

  signals:
    // --- Generic Commands ---

    void EnableDebugMode(const bool& enabled);

    void CommandMotionMode(const Motion::MotionMode& mode);
    void EnableReactiveCw(const bool& enabled);
    void UpdateReactiveCwBounds(const double& lower_bound,
                                const double& upper_bound);

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

    // HEBI

    void CommandHebi(const std::vector<double>& deg);
    void LoadHebiGains(const QString& file_path);
    void LoadHebiSafety(const QString& file_path);

#if LIBRA_VERSION == 2
    void EnablePitchRelease(const bool& enabled);

    void StartHebiLevelCal();
    void StartHebiTwoAngleCal(const double& deg_a, const double& deg_b);
    void StartHebiFrictionCal(const double& deg_a, const double& deg_b);
    void CancelHebiCal();

    void EnableModelLogging(const bool& enabled);
#endif

    // EPOS (Maxon)

#if LIBRA_VERSION == 2
    void ConnectEpos();
    void CommandEpos(const std::vector<double>& deg);
#endif

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar (menu_bar.cpp) ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);
    void on_a_reset_window_size_triggered();

    // Automation Menu

    void on_a_reactive_cw_bounds_triggered();
    void on_a_reactive_cw_ignore_water_level_triggered(bool checked);

    // Trajectory Menu

    void on_a_trajectory_load_triggered();
    void on_a_trajectory_save_triggered();

    // Motion Control Menu

#if LIBRA_VERSION == 2
    void on_a_motion_handoff_band_triggered();
    void on_a_motion_inertia_ff_toggled(bool checked);

    void on_a_motion_calibrate_level_triggered();
    void on_a_motion_calibrate_two_angle_triggered();
    void on_a_motion_calibrate_friction_triggered();
    void on_a_motion_calibrate_cancel_triggered();

    void on_a_motion_calibration_save_triggered();
    void on_a_motion_calibration_load_triggered();
    void on_a_motion_calibration_clear_triggered();

    void on_a_motion_model_logging_toggled(bool checked);
#endif

    // Actuators Menu

#if LIBRA_VERSION == 2
    void on_a_epos_connect_triggered();
    void on_a_eta_extension_triggered();
#endif

    void on_a_hebi_load_gains_triggered();
    void on_a_hebi_load_safety_triggered();
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

    void on_a_water_set_mass_triggered();

    // Quick Actions Menu

    void on_a_connect_all_triggered();
    void on_a_disconnect_all_triggered();

    // --- Main Window (various files) ---

    // Arm (arm.cpp)

    void on_pb_arm_start_clicked();
    void on_pb_arm_stop_clicked();
    void on_pb_arm_return_clicked();
    void on_cb_motion_mode_activated(int index);
#if LIBRA_VERSION == 2
    void on_pb_release_pitch_toggled(bool checked);
#endif

    // Trajectory (trajectory.cpp)

    void on_pb_trajectory_add_clicked();
    void on_pb_trajectory_clear_clicked();
    void on_pb_trajectory_play_clicked();

    // Water (water.cpp)

    void on_pb_water_fill_A_clicked(bool checked);
    void on_pb_water_drain_A_clicked(bool checked);
#if LIBRA_VERSION == 1
    void on_pb_water_fill_B_clicked(bool checked);
    void on_pb_water_drain_B_clicked(bool checked);
#endif

    // Manipulator (manipulator.cpp)

#if LIBRA_VERSION == 1
    void on_pb_manip_slow_clicked();
    void on_pb_manip_fast_clicked();
#endif

    // Camera (camera.cpp)

    void on_cb_camera_name_currentTextChanged(const QString& sel);
    void on_pb_camera_capture_clicked();
    void on_pb_camera_record_clicked();
    void on_pb_camera_flip_h_clicked();
    void on_pb_camera_flip_v_clicked();

    // --- Uncategorized ---

  private:
    // NOLINTEND

    // --- Helper Functions (main_window.cpp) ---

    void ConfigureUi();
    void InitializeElementMaps();
    void InitializeDeviceManagers();
    void InitializeThreads();

    void SetMotionMode(const Motion::MotionMode& mode);

    void UpdateActuatorControls();
    void UpdateModeControls();
#if LIBRA_VERSION == 2
    void UpdateMotionControls();
#endif

    // --- Helper Functions (arm.cpp) ---

    void SendJointTargets(const std::vector<double>& targets);

    // --- Helper Functions (trajectory.cpp) ---

    void UpdateTrajectoryControls();

    void SetTrajectoryPlaying(const bool& playing);
    void PlayNextTrajectoryStep();
    bool ConfirmTrajectoryStep();
    [[nodiscard]] QString GetTrajectoryStepText() const;

    // --- Data Members ---

    Ui::MainWindow* ui_{nullptr};

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{true};

    QSize default_size_{};  // main UI window size

    FeedbackElementMapOfMaps feedback_element_map_;
    std::unordered_map<Joint::Name, QDoubleSpinBox*> target_spin_box_map_;

    // Arm

    Motion::MotionMode motion_mode_{Motion::MotionMode::kManual};
    bool arm_moving_{false};        // any actuator moving
    bool populate_targets_{false};  // set on actuator connect

    // Targets as joint angles, indexed by `Joint::Name` (deg)
    std::vector<double> current_target_;   // most recently sent
    std::vector<double> previous_target_;  // second most recently sent

    struct ArmLimits {
        double pitch_min{-180.0};
        double pitch_max{180.0};
#if LIBRA_VERSION == 1
        double roll_min{-180.0};
        double roll_max{180.0};
        double j1_min{-180.0};
        double j1_max{180.0};
        double j2_min{-180.0};
        double j2_max{180.0};
        double j3_min{-180.0};
        double j3_max{180.0};
#elif LIBRA_VERSION == 2
        double yaw_min{-180.0};
        double yaw_max{180.0};
#endif
    } original_limits_;

    // Trajectory

    std::vector<std::vector<double>>
        trajectory_;  // poses in `Joint::Name` order (deg)
    QString trajectory_file_;

    std::optional<std::size_t> trajectory_step_;  // none = not started
    bool trajectory_playing_{false};
    QTimer* step_delay_timer_{nullptr};  // settle time after each step

    // Water

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

    // Motion Control

    std::shared_ptr<Libra2MotionController> motion_controller_;  // pitch joint
#endif
};
