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
#include <libudev.h>
// Project Headers
#include "open_epos_window.h"
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

    // コンソールを用意
    // Prepare console
    // clang-format off
    std::cout <<  // note: do NOT mess with the spacing!
        "\n"
        " 888      8888888 888888b.   8888888b.         d8888             d8888 \n"
        " 888        888   888  \"88b  888   Y88b       d88888            d88888 \n"
        " 888        888   888  .88P  888    888      d88P888           d88P888 \n"
        " 888        888   8888888K.  888   d88P     d88P 888          d88P 888 88888b.  88888b. \n"
        " 888        888   888  \"Y88b 8888888P\"     d88P  888         d88P  888 888 \"88b 888 \"88b \n"
        " 888        888   888    888 888 T88b     d88P   888        d88P   888 888  888 888  888 \n"
        " 888        888   888   d88P 888  T88b   d8888888888       d8888888888 888 d88P 888 d88P \n"
        " 88888888 8888888 8888888P\"  888   T88b d88P     888      d88P     888 88888P\"  88888P\" \n"
        "                                                                       888      888 \n"
        "                                                                       888      888 \n"
        "                                                                       888      888 \n"
        << std::endl;
    // clang-format on

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

    // 利用可能なポートのスキャン
    // Scan for available device ports
    std::cout << "\n----- Device List -----\n\n";
    PrintDeviceList();

    // SerialWaterのポートをユーザーが指定できるようにする
    // Allow user to specify SerialWater port
    std::cout << "Specify the port to be used by SerialWater:\n";
    ser_water_ = std::make_unique<Serial>("water", in.readLine().toStdString());

    // SerialServoのポートをユーザーが指定できるようにする
    // Allow user to specify SerialServo port
    std::cout << "Specify the port to be used by SerialServo:\n";
    ser_servo_ = std::make_unique<Serial>("servo", in.readLine().toStdString());

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
MainWindow::~MainWindow() {}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief TODO
 */
void MainWindow::PrintDeviceList() {
    // Create udev context
    struct udev* udev = udev_new();
    if (!udev) {
        std::cerr << "[ERROR] Unable to create udev context" << std::endl;
        return;
    }

    // Initialize udev enumerator
    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    // Get list of devices
    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* entry;

    // Iterate through list
    udev_list_entry_foreach(entry, devices) {
        const char* path = udev_list_entry_get_name(entry);
        struct udev_device* device = udev_device_new_from_syspath(udev, path);

        // Retrieve device information
        const char* deviceNode = udev_device_get_devnode(device);
        const char* deviceName = udev_device_get_sysname(device);

        std::cout << "Device Node: " << (deviceNode ? deviceNode : "unknown")
                  << std::endl;
        std::cout << "Device Name: " << (deviceName ? deviceName : "unknown")
                  << std::endl;
        std::cout << std::endl;

        // Free current device before moving on to next one
        udev_device_unref(device);
    }

    // Clean up
    udev_enumerate_unref(enumerate);
    udev_unref(udev);
}

/**
 * @brief TODO
 */
void MainWindow::ConnectSerial() {}

/**
 * @brief TODO
 */
void MainWindow::ConnectHebi() {}

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
void MainWindow::on_a_serial_triggered() {}
