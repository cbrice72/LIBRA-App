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
#include <iostream>
// Other Libraries' Headers
//   Qt
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

// constexpr uint example = 0;

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget (default: nullptr)
 */
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow) {
    ui_->setupUi(this);

    // --- Arduino Connection (via serial USB) ---

    // HEBI�A�N�`���G�[�^���ڑ�
    // Connect HEBI actuators
    libra_arm_ = std::make_unique<LibraHebi>();
    if (!libra_arm_->Connect()) {
        qWarning() << "[WARN] Initializing without HEBI actuators.";
    }

    // --- Thread Management ---

    // TOOD: is this necessary in the new app?

    /* In Qt, thread management for subclassed QThreads generally has 4 steps:
     *   1) Initialize a new QThread object
     *   2) Register a QThread signal to return data to a MainWindow handler
     *   3) Register the QThread's exit signal to its own destruction slot
     *   4) Spin off the QThread
     * If the thread loops continuously, there is a fifth step:
     *   5) In the MainWindow destructor, interrupt or forcibly stop the QThread
     */

    // Controller status thread
    /*
    status_thread_ = new StatusThread(handle_, start, count);

    connect(status_thread_, &StatusThread::StatusReady,  // When thread has data
            this, &MainWindow::UpdateStatus);            // ... hand off to main
    connect(status_thread_, &StatusThread::finished,     // When thread exits
            status_thread_, &StatusThread::deleteLater);  // ... deallocate it

    status_thread_->start();
    */
}

/**
 * @brief Standard destructor.
 */
MainWindow::~MainWindow() {
    // TODO
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// !Worker Threads
//------------------------------------------------------------------------------

/**
 * @brief Primary request and response loop for the LIBRA app.
 *        Continuously retrieves actuator and Arduino statuses.
 *        Executed when `start()` is called on the thread.
 *
 * @todo is this necessary in the new app?
 */
/*
void MainThread::run() {
    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // TODO
    }
}
*/

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
    // TODO
}

/**
 * @brief Event handler for "EPOS" menu bar action "Disonnect".
 *        Terminates the active EPOS controller connection, if any.
 */
void MainWindow::on_a_epos_disconnect_triggered() {
    // TODO
}

/**
 * @brief Event handler for "HEBI" menu bar action "Connect".
 *        Brings up a dialog box for inputting an IP address and actuator names.
 */
void MainWindow::on_a_hebi_connect_triggered() {
    // Display the "Connect to HEBI" dialog
    // TODO

    // Only continue if "Connect" was successful
    // TODO

    /*
    if (debug_mode_) {
        qDebug() << "[DEBUG] HEBI serial dialog returned successfully"
    }
    */

    // Open a connection to the HEBI actuators
    // TODO
}

/**
 * @brief Event handler for "HEBI" menu bar action "Disconnect".
 *        Terminates all active HEBI actuator connections, if any.
 */
void MainWindow::on_a_hebi_disconnect_triggered() {
    libra_arm_.reset();
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
 * @brief TODO
 */
void MainWindow::on_pb_arm_start_clicked() {}

/**
 * @brief TODO
 */
void MainWindow::on_pb_arm_stop_clicked() {}

/**
 * @brief TODO
 */
void MainWindow::on_pb_arm_convert_clicked() {}

//------------------------------------------------------------------------------
// !Pumps
//------------------------------------------------------------------------------

/**
 * @brief TODO
 */
void MainWindow::on_pb_pumps_enable_clicked() {}

/**
 * @brief TODO
 */
void MainWindow::on_pb_pumps_disable_clicked() {}

/**
 * @brief TODO
 */
void MainWindow::on_pb_pumps_drain_clicked() {}

//------------------------------------------------------------------------------
// !Camera
//------------------------------------------------------------------------------

/**
 * @brief TODO
 */
void MainWindow::on_pb_camera_slow_clicked() {}

/**
 * @brief TODO
 */
void MainWindow::on_pb_camera_fast_clicked() {}

//------------------------------------------------------------------------------
// !Misc.
//------------------------------------------------------------------------------

/**
 * @brief TODO
 */
void MainWindow::on_pb_logshot_clicked() {}

//------------------------------------------------------------------------------
// !Uncategorized
//------------------------------------------------------------------------------
