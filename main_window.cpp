/******************************************************************************
 * @file   main_window.cpp
 * @brief  Main app window implementation file.
 *
 * @author brice.c.aa
 * @date   2024/2/22
 ******************************************************************************/

// Related Header
#include "main_window.h"
// C++ Standard Library Headers
#include <filesystem>
#include <iostream>
// Other Libraries' Headers
//   Qt
#include <QDateTime>
#include <QMessageBox>
// Project Headers
// #include "open_epos_window.h"  // TODO(brice.c.aa): add EPOS4 motor control
#include "serial_dialog.h"
#include "ui_main_window.h"

// #include "utility.h"

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Menu Bar
 * !Main Window
 * !Input
 * !Pumps
 * !Camera
 * !Misc.
 * !Uncategorized
 */

/* Constants */

constexpr uint kHebiNodeCount = 5;      // total no. of HEBI actuators
constexpr uint kHebiFeedbackCount = 3;  // total no. of actuator feedback types

constexpr uint kMaxonNodeCount = 1;  // total no. of Maxon (EPOS) actuators

constexpr uint kFluidStateCount = 4;  // no. of pumps * no. of pump states

constexpr uint kCameraNodeCount = 3;  // total no. of camera servos

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget (default: nullptr)
 */
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow) {
    ui_->setupUi(this);

    // --- Arduino Connection (via serial USB) ---

    // TODO: implementation

    // --- Thread Management ---

    /* In Qt, thread management for subclassed QThreads generally has 4 steps:
     *   1) Initialize a new QThread object
     *   2) Register a QThread signal to return data to a MainWindow handler
     *   3) Register the QThread's exit signal to its own destruction slot
     *   4) Spin off the QThread
     * If the thread loops continuously, there is a fifth step:
     *   5) In the MainWindow destructor, interrupt or forcibly stop the QThread
     */

    // TODO: documentation "... thread"
    hebi_thread_ = new HebiThread();

    connect(hebi_thread_, &HebiThread::InformState,   // When thread has data
            this, &MainWindow::UpdateHebi);           // ... hand off to main
    connect(this, &MainWindow::DisconnectHebi,        // When main sends command
            hebi_thread_, &HebiThread::Stop);         // ... directly call thread
    connect(hebi_thread_, &HebiThread::finished,      // When thread exits
            hebi_thread_, &HebiThread::deleteLater);  // ... deallocate it

    hebi_thread_->start();

    // TODO: documentation "... thread"
    pump_thread_ = new PumpThread();

    /*
    connect(pump_thread_, &PumpThread::SignalName,    // signal
            this, &MainWindow::SomeFunction2);        // slot
    */
    connect(pump_thread_, &PumpThread::finished,      // signal
            pump_thread_, &PumpThread::deleteLater);  // slot

    pump_thread_->start();

    // --- Logging Initialization ---

    std::filesystem::create_directory("log");

    auto dts = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss").toStdString();
    continuous_log_.open("log/" + dts + "_continuous_log.csv");
    continuous_log_
        << "Time,,"
        << "TP_Roll (deg),TP_Pitch (deg),TP_J1 (deg),TP_J2 (deg),TP_J3 (deg),,"
        << "PP_Roll (deg),PP_Pitch (deg),PP_J1 (deg),PP_J2 (deg),PP_J3 (deg),,"
        << "PT_Roll (Nm),PT_Pitch (Nm),PT_J1 (Nm),PT_J2 (Nm),PT_J3 (Nm),,"
        << "A_IN,B_IN,A_OUT,B_OUT,,"
        << "TP_CamBase (deg),TP_CamPan (deg),TP_CamTilt (deg)\n";

    snapshot_log_.open("log/" + dts + "_shot_log.csv");
    snapshot_log_
        << "Time,,"
        << "TP_Roll (deg),TP_Pitch (deg),TP_J1 (deg),TP_J2 (deg),TP_J3 (deg),,"
        << "PP_Roll (deg),PP_Pitch (deg),PP_J1 (deg),PP_J2 (deg),PP_J3 (deg),,"
        << "PT_Roll (Nm),PT_Pitch (Nm),PT_J1 (Nm),PT_J2 (Nm),PT_J3 (Nm),,"
        << "Voltage (V),Current (A)\n";
}

/**
 * @brief Standard destructor.
 */
MainWindow::~MainWindow() {
    // Wrap up the worker threads gracefully
    hebi_thread_->requestInterruption();  // signal thread to stop looping
    hebi_thread_->wait();                 // wait for thread cleanup to finish
    pump_thread_->requestInterruption();
    pump_thread_->wait();

    // Close log files
    if (continuous_log_.is_open()) {
        continuous_log_.close();
    }
    if (snapshot_log_.is_open()) {
        snapshot_log_.close();
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Provides a formatted string of the current date and time.
 *
 * @return std::string Formatted as "yyyy-MM-ddTHH:mm:ss.zzz"
 */
QString MainWindow::GetDateTimeString() {
    return QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
}

/**
 * @brief Updates the actual position and torque values shown in the UI for the
 *        HEBI actuators. Used together with the signal HebiThread::InformState().
 * 
 * @param pos Positional values reported by each HEBI actuator.
 * @param torque Torque values reported by each HEBI actuator.
 */
void MainWindow::UpdateHebi(std::array<double, 5> pos, std::array<double, 5> torque) {
    // Update actual position
    ui_->l_actual_roll->setText(QString::number(pos.at(0)));
    ui_->l_actual_pitch->setText(QString::number(pos.at(1)));
    ui_->l_actual_j1->setText(QString::number(pos.at(2)));
    ui_->l_actual_j2->setText(QString::number(pos.at(3)));
    ui_->l_actual_j3->setText(QString::number(pos.at(4)));

    // Update actual torque
    ui_->l_torque_roll->setText(QString::number(torque.at(0)));
    ui_->l_torque_pitch->setText(QString::number(torque.at(1)));
    ui_->l_torque_j1->setText(QString::number(torque.at(2)));
    ui_->l_torque_j2->setText(QString::number(torque.at(3)));
    ui_->l_torque_j3->setText(QString::number(torque.at(4)));
}

//------------------------------------------------------------------------------
// !Worker Threads
//------------------------------------------------------------------------------

// TODO: move this all to a slot in MainWindow and instead send a
//       signal to MainWindow every `if (count_ == 0)` for logging
/*
// Timestamp
continuous_log_ << GetDateTimeString() + ",,";

// Actuator info
for (auto i = 0; i < kHebiFeedbackCount; i++) {
    for (auto j = 0; j < kHebiNodeCount; j++) {
        continuous_log_ << value_.at(j).at(i) << ",";
    }
    continuous_log_ << ",";
}

// Fluid system info
for (auto i = 0; i < kFluidStateCount; i++) {
    continuous_log_ << ((water_cmd & (1 << (3 - i))) ? 1 : 0)
                    << ",";
}
continuous_log_ << ",";

// Camera actuator info
continuous_log_ << camera_pos_.at(0) << "," << camera_pos_.at(1)
                << "," << camera_pos_.at(2) << "\n";
*/

/**
 * @brief TODO: documentation
 */
void PumpThread::run() {
    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // TODO: implementation

        QThread::msleep(25);  // update 4 times/second (arbitrary)
    }
}

//------------------------------------------------------------------------------
// !Menu Bar
//------------------------------------------------------------------------------

void MainWindow::on_a_debug_mode_toggled(bool checked) {
    debug_mode_ = checked;
}

/**
 * @brief Event handler for "EPOS" menu bar action "Connect".
 *        Brings up a dialog box similar to `VCS_OpenDeviceDlg()`.
 */
void MainWindow::on_a_epos_connect_triggered() {
    // TODO: implementation
}

/**
 * @brief Event handler for "EPOS" menu bar action "Disonnect".
 *        Terminates the active EPOS controller connection, if any.
 */
void MainWindow::on_a_epos_disconnect_triggered() {
    // TODO: implementation
}

/**
 * @brief Event handler for "HEBI" menu bar action "Connect".
 *        Brings up a dialog box for inputting an IP address and actuator names.
 */
void MainWindow::on_a_hebi_connect_triggered() {
    // Display the "Connect to HEBI" dialog
    // TODO: implementation

    // Only continue if "Connect" was successful
    // TODO: implementation

    /*
    if (debug_mode_) {
        qDebug() << "[DEBUG] HEBI serial dialog returned successfully"
    }
    */

    // Open a connection to the HEBI actuators
    // TODO: implementation
}

/**
 * @brief Event handler for "HEBI" menu bar action "Disconnect".
 *        Terminates all active HEBI actuator connections, if any.
 */
void MainWindow::on_a_hebi_disconnect_triggered() {
    emit DisconnectHebi();
}

/**
 * @brief Event handler for "Pumps" menu bar action "Connect".
 *        Brings up a dialog box of available serial USB devices.
 */
void MainWindow::on_a_pumps_connect_triggered() {
    // Display the "Connect to Serial" dialog
    SerialDialog w_serial("USB");
    w_serial.setModal(true);
    w_serial.exec();

    // Only continue if "Connect" was successful
    if (w_serial.result() != QDialog::Accepted) {
        qWarning() << "[WARN] Failed to connect to serial pump controller!";
        return;
    }

    if (debug_mode_) {
        qDebug() << "[DEBUG] Pumps serial dialog returned successfully";
    }

    // Open a connection to the "SerialWater" Arduino
    ser_water_ = std::make_unique<Serial>("SerialWater",
                                          "/dev/" + w_serial.GetDeviceName());
}

/**
 * @brief Event handler for "Pumps" menu bar action "Disconnect".
 *        Terminates the connection to the `SerialWater` Arduino, if it exists.
 */
void MainWindow::on_a_pumps_disconnect_triggered() {
    ser_water_.reset();
}

/**
 * @brief Event handler for "Camera" menu bar action "Connect".
 *        Brings up a dialog box of available serial USB devices.
 */
void MainWindow::on_a_camera_connect_triggered() {
    // Display the "Connect to Serial" dialog
    SerialDialog w_serial("USB");
    w_serial.setModal(true);
    w_serial.exec();

    // Only continue if "Connect" was successful
    if (w_serial.result() != QDialog::Accepted) {
        qWarning()
            << "[WARN] Failed to connect to serial camera servo controller!";
        return;
    }

    if (debug_mode_) {
        qDebug() << "[DEBUG] Camera serial dialog returned successfully";
    }

    // Open a connection to the "SerialServo" Arduino
    ser_servo_ = std::make_unique<Serial>("SerialServo",
                                          "/dev/" + w_serial.GetDeviceName());
}

/**
 * @brief Event handler for "Camera" menu bar action "Disconnect".
 *        Terminates the connection to the `SerialServo` Arduino, if it exists.
 */
void MainWindow::on_a_camera_disconnect_triggered() {
    ser_servo_.reset();
}

/**
 * @brief Event handler for "LIDAR" menu bar action "Connect".
 *        Brings up a dialog box of available serial ACM devices.
 *
 * @note Although the Hokuyo LIDAR is connected via USB, it is listed as ACM.
 *       See https://sourceforge.net/p/urgnetwork/wiki/serial_linux_en/
 */
void MainWindow::on_a_lidar_connect_triggered() {
    // Display the "Connect to Serial" dialog
    SerialDialog w_serial("ACM", debug_mode_);
    w_serial.setModal(true);
    w_serial.exec();

    // Only continue if "Connect" was successful
    if (w_serial.result() != QDialog::Accepted) {
        qWarning()
            << "[WARN] Failed to connect to serial camera servo controller!";
        return;
    }

    if (debug_mode_) {
        qDebug() << "[DEBUG] LIDAR serial dialog returned successfully";
    }

    // Open a connection to the Hokuyo LIDAR
    lidar_ = std::make_unique<LibraLidar>(w_serial.GetDeviceName());
}

/**
 * @brief Event handler for "LIDAR" menu bar action "Disconnect".
 *        Terminates the connection to the LIDAR, if it exists.
 */
void MainWindow::on_a_lidar_disconnect_triggered() {
    lidar_.reset();
}

/**
 * @brief Event handler for "LIDAR" menu bar action "About".
 *        Opens a dialog box with sensor metadata, if connected.
 */
void MainWindow::on_a_lidar_about_triggered() {
    if (lidar_ != nullptr) {
        QMessageBox::information(this, "LIDAR - About",
                                 QString::fromStdString("<pre>"  // monospace
                                                        + lidar_->GetMetadata()
                                                        + "</pre>"));
    } else {
        qWarning() << "[WARN] LIDAR object not yet initialized!";
    }
}

//------------------------------------------------------------------------------
// !Main Window
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Input
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_start_clicked() {
    // TODO: adapt code
    /*
    input_.at(0) = ibox_roll_->GetNum();
    input_.at(1) = ibox_pitch_->GetNum();
    input_.at(2) = ibox_j1_->GetNum();
    input_.at(3) = ibox_j2_->GetNum();
    input_.at(4) = ibox_j3_->GetNum();

    // Only move arm when fluid system isn't running
    if (water_mode_ == WaterMode::kStandby) {
        // Begin arm movement
        libra_arm_->Move(input_.at(0), input_.at(1), input_.at(2),
                         input_.at(3), input_.at(4));
    }
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_stop_clicked() {
    // TODO: adapt code
    /*
    libra_arm_->Stop();
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_convert_clicked() {
    // TODO: adapt code
    /*
    // NOLINTBEGIN(readability-identifier-length): equation variables

    const double r = ibox_r_->GetNum();
    const double theta = ibox_theta_->GetNum();
    const double L = 989;
    const double L_hand = 1014;
    const double a = L;
    const double b = L + L_hand;

    const double k = (r * r + a * a - b * b) / (2 * a);
    const double alpha = (atan2(0, r) + atan2(sqrt(r * r - k * k), k));
    const double beta = asin(r * sin(alpha) / b);

    // NOLINTEND(readability-identifier-length): equation variables

    ibox_j1_->SetNum(theta + alpha / M_PI * 180);
    ibox_j2_->SetNum(-180 + beta / M_PI * 180);
    ibox_j3_->SetNum(0);
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_r_plus_clicked() {
    // TODO: adapt code
    /*
    ibox_r_->SetNum(ibox_r_->GetNum() + ibox_increment_->GetNum());
    OnClick(btn_convert_);
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_r_minus_clicked() {
    // TODO: adapt code
    /*
    ibox_r_->SetNum(ibox_r_->GetNum() - ibox_increment_->GetNum());
    OnClick(btn_convert_);
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_theta_plus_clicked() {
    // TODO: adapt code
    /*
    ibox_r_->SetNum(ibox_r_->GetNum() - ibox_increment_->GetNum());
    OnClick(btn_convert_);
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_theta_minus_clicked() {
    // TODO: adapt code
    /*
    ibox_theta_->SetNum(ibox_theta_->GetNum() - ibox_increment_->GetNum()
                        / ibox_r_->GetNum() * 180 / M_PI);
    OnClick(btn_convert_);
    */
}

//------------------------------------------------------------------------------
// !Pumps
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_pumps_enable_clicked() {
    // TODO: adapt code
    /*
    water_en_ = true;
    water_mode_ = WaterMode::kStandby;
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_pumps_disable_clicked() {
    // TODO: adapt code
    /*
    water_en_ = false;
    water_mode_ = WaterMode::kStandby;
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_pumps_drain_clicked() {
    // TODO: adapt code
    /*
    water_en_ = true;
    water_mode_ = WaterMode::kDrain;
    */
}

//------------------------------------------------------------------------------
// !Camera
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_camera_slow_clicked() {
    // TODO: adapt code
    /*
    camera_setpos_.at(1) = ui_->sb_camera_pan->value();
    camera_setpos_.at(2) = ui_->sb_camera_tilt->value();
    camera_dir_.at(1) = (camera_setpos_.at(1) >= camera_pos_.at(1)) ? 1 : -1;
    camera_dir_.at(2) = (camera_setpos_.at(2) >= camera_pos_.at(2)) ? 1 : -1;
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_camera_fast_clicked() {
    // TODO: adapt code
    /*
    camera_pos_.at(1) = ui_->sb_camera_pan->value();
    camera_pos_.at(2) = ui_->sb_camera_tilt->value();
    */
}

//------------------------------------------------------------------------------
// !Misc.
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_logshot_clicked() {
    // TODO: implementation
    /*
    auto dts = GetDateTimeString();
    snapshot_log_ << dts.toStdString() << ",,";
    for (auto i = 0; i < kHebiFeedbackCount; i++) {
        for (auto j = 0; j < kHebiNodeCount; j++) {
            snapshot_log_ << value_.at(j).at(i) << ",";
        }
        snapshot_log_ << ",";
    }
    snapshot_log_ << ui_->l_voltage->text() << ","
                  << ui_->l_current->text() << "\n";

    QDebug() << "Snapshot - " << dts << " | Voltage: " << ui_->l_voltage->text()
             << " V | Current: " << ui_->l_current->text() << " A\n";
    */
}

//------------------------------------------------------------------------------
// !Uncategorized
//------------------------------------------------------------------------------
