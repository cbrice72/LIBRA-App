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
//   (none)
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

    QTextStream in(stdin);  // used to retrieve user input via readLine()

    // HEBIアクチュエータを接続
    // Connect HEBI actuators
    libra_arm_ = std::make_unique<LibraHebi>();
    if (!libra_arm_->Connect()) {
        qDebug()
            << "Failed to connect to HEBI actuators; continue anyways? [y/n]:";
        if (in.readLine() == "n") {
            // HEBIアクチュエータに接続できない場合は、プログラムを終了
            // Exit app if connection to HEBI actuators can't be established
            std::cout << "Exiting...\n";
            QThread::sleep(1);  // give user time to read message
            return;
        }
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

/**
 * @brief TODO
 */
void MainWindow::on_a_epos_triggered() {}

/**
 * @brief TODO
 */
void MainWindow::on_a_hebi_triggered() {}

/**
 * @brief TODO
 */
void MainWindow::on_a_pumps_triggered() {
    // Display the "Open EPOS device" dialog
    SerialDialog w_serial;
    w_serial.setModal(true);
    w_serial.exec();

    // Only continue if "Connect" was successful
    if (w_serial.result() != QDialog::Accepted) {
        qWarning() << "Failed to connect to serial pump controller!";
        return;
    }

    // Save off the port handle (TODO)
    // handle_ = w_open_epos.GetEPOSHandle();
}

/**
 * @brief TODO
 */
void MainWindow::on_a_camera_triggered() {}
