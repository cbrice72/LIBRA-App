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
#include <QLabel>       // Qt::Widgets
#include <QMainWindow>  // Qt::Widgets

// Project Headers
#include "epos_thread.h"
#include "hebi_thread.h"

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

    void CommandEpos(const std::vector<double>& deg);
    void CommandHebi(const std::vector<double>& deg);

    // NOLINTBEGIN: Qt-generated
  private slots:
    // --- Menu Bar ---

    // Preferences Menu

    void on_a_debug_mode_toggled(bool checked);

    // EPOS menu

    // NOTE: signals connected directly to thread slots

    // HEBI menu

    // NOTE: signals connected directly to thread slots

    // --- Main Window ---

    void on_pb_connect_all_clicked();
    void on_pb_disconnect_all_clicked();

    void on_pb_yaw_start_clicked();
    void on_pb_pitch_start_clicked();
    void on_pb_arm_start_clicked();

    // --- Uncategorized ---

  private:
    // NOLINTEND

    // --- Helper Functions ---

    void InitializeFeedbackElementMap();

    // --- Data Members ---

    Ui::MainWindow* ui_;
    EposThread* epos_thread_;
    HebiThread* hebi_thread_;

    bool debug_mode_{true};

    FeedbackElementMapOfMaps feedback_element_map_;
};
