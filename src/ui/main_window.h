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
#include "arduino_manager.h"
#include "camera_manager.h"
#include "hebi_thread.h"
#include "logger.h"
#if LIBRA_VERSION == 2
# include "epos_thread.h"
#endif

QT_BEGIN_NAMESPACE

namespace Ui {  // NOLINT: Qt-generated
class MainWindow;
}  // namespace Ui

QT_END_NAMESPACE

/**
 * @brief LIBRA joint names (not to be confused with -actuator- names).
 *
 * @note In most cases, a joint name will be identical to the actuator's name.
 *       However, more complex joints (those driven by multiple actuators, such
 *       as a differential drive) will differ in names. Since the user commands
 *       joint angles -- NOT actuator angles -- a distinction must be made.
 *
 * @note To keep logic simple, joints with the same name as their corresponding
 *       actuator are defined first. For all other joints, let the compiler
 *       decide what the enum resolves to.
 *
 * @see Actuator::Name
 */
enum Joint {
#if LIBRA_VERSION == 1
    kJ1 = Actuator::Name::kJ1,
    kJ2 = Actuator::Name::kJ2,
    kJ3 = Actuator::Name::kJ3,
    kRoll,
    kPitch,
#elif LIBRA_VERSION == 2
    kYaw = Actuator::Name::kYaw,
    kPitch = Actuator::Name::kPitch
#endif
};

// Type alias for conveniently accessing a feedback label, where rows are
// joints and columns are feedback types (order follows MainWindow UI layout)
using FeedbackElementMapOfMaps =
    std::unordered_map<Joint, std::unordered_map<Actuator::Feedback, QLabel*>>;

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
        const std::unordered_map<Actuator::Name, double>& feedbacks,
        const Actuator::Feedback feedback_type);
    void HandleActuatorStatus(const QString& status, const Actuator::Type type);

  signals:
    // --- Generic Commands ---

    void EnableDebugMode(const bool& enabled);

    void EnableAutoTorqueComp(const bool& enabled);

    // --- Arduino Commands ---

    void ConnectWater(const QString& port_name);
    void CommandWater(const Water::Side& side, const Water::State& state);

#if LIBRA_VERSION == 1
    void ConnectManip(const QString& port_name);
    void CommandManip(const double& pan, const double& tilt,
                      const bool& move_slow);
    void InformPitch(const double& angle);
#endif

    // --- Actuator Commands ---

    void CommandHebi(const std::vector<double>& deg);
    void UpdateHebiTorqueCompBounds(const double& upper_bound,
                                    const double& lower_bound);
#if LIBRA_VERSION == 2
    void CommandEpos(const std::vector<double>& deg);
#endif

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);

    // Actuators Menu

    void on_a_hebi_override_limits_toggled(bool checked);
    void on_a_hebi_central_torque_comp_triggered();

    // Sensors Menu
    // (most actions are handled via signals, and thus don't need functions)

    void on_a_refresh_camera_list_triggered();

    // Water Menu

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
    void on_pb_autocomp_enable_toggled(bool checked);

    // Water

    void on_pb_water_fill_A_toggled(bool checked);
    void on_pb_water_drain_A_toggled(bool checked);
#if LIBRA_VERSION == 1
    void on_pb_water_fill_B_toggled(bool checked);
    void on_pb_water_drain_B_toggled(bool checked);
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

    void ConnectWaterHelper();
#if LIBRA_VERSION == 1
    void ConnectManipHelper();
#endif

    // --- Data Members ---

    Ui::MainWindow* ui_{nullptr};

    std::unique_ptr<Logger> logger_;
    bool debug_mode_{true};

    FeedbackElementMapOfMaps feedback_element_map_;

    // Device Managers

    ArduinoManager* arduino_manager_{nullptr};       // serial device control
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
