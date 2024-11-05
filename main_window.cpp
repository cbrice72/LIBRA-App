/******************************************************************************
 * @file   main_window.cpp
 * @brief  Main app window implementation file.
 *
 * @author brice.c.aa
 ******************************************************************************/

// Related Header
#include "main_window.h"
// C++ Standard Library Headers
#include <filesystem>
#include <iostream>
#include <sys/stat.h>
// Other Libraries' Headers
//   Qt
#include <QDateTime>
#include <QMessageBox>
#include <QSerialPortInfo>
// Project Headers
// #include "open_epos_window.h"  // TODO(brice.c.aa): add EPOS4 motor control
#include "serial_dialog.h"
#include "ui_main_window.h"

// #include "utility.h"

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Menu Bar
 * !Main Window
 * !Arm
 * !Counterweight
 * !Camera
 * !Misc.
 * !Uncategorized
 */

/* Constants */

constexpr int kInfoLifespan = 4000;  // 4s timer for non-hover status tips

constexpr int kHebiNodeCount = 5;      // total number of HEBI actuators
constexpr int kHebiFeedbackCount = 3;  // total number of actuator feedback types
constexpr int kFluidStateCount = 4;  // number of pumps * number of pump states
constexpr int kCameraNodeCount = 3;  // total number of camera servos

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget (default: nullptr)
 */
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow),
      ser_water_(std::make_shared<QSerialPort>(this)) {
    ui_->setupUi(this);

    // Set UI elements
    ui_->a_debug_mode->setChecked(debug_mode_);

    // Check if app is running in Windows Subsystem for Linux (WSL2)
    {
        const char* wsl_path = "/run/WSL";

        struct stat buf {};

        if (stat(wsl_path, &buf) == 0 && S_ISDIR(buf.st_mode)) {
            qWarning()
                << "[WARN] You seem to be running this on WSL2. Please ensure "
                   "you have properly forwarded your USB connections.";
        }
    }

    // --- Component Connection ---

    // TODO: EPOS autoconnect

    // TODO: LibraHebi autoconnect

    // TODO: SerialWater autoconnect

    // Camera
    qDebug() << "[INFO] Checking available video inputs...";

    const auto cameras = QMediaDevices::videoInputs();
    for (const auto& camera_device : cameras) {
        auto id = QString(camera_device.id());

        if (debug_mode_) {
            qDebug() << "[DEBUG] Found camera at " << camera_device.id();
        }

        // Populate ComboBox (new items are appended to existing list)
        ui_->cb_camera_id->addItem(id);
        // Populate internal map (used on ComboBox change)
        available_cameras_[id] = camera_device.description();
    }

    if (!available_cameras_.empty()) {
        ui_->pb_camera_capture->setEnabled(true);
        ui_->pb_camera_record->setEnabled(true);
    } else if (debug_mode_) {
        qDebug() << "[DEBUG] No cameras were found";
    }

    // TODO: SerialServo autoconnect

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

    // Main controller thread
    main_thread_ = new MainThread(libra_arm_, ser_water_, ser_servo_);

    connect(this, &MainWindow::CommandArm,  // send arm commands to main thread
            main_thread_, &MainThread::UpdateArmTarget);
    connect(main_thread_,
            &MainThread::finished,  // when thread exits, deallocate
            main_thread_, &MainThread::deleteLater);

    main_thread_->start();
}

/**
 * @brief Standard destructor.
 */
MainWindow::~MainWindow() {
    // Wrap up the worker thread(s) gracefully
    main_thread_->requestInterruption();  // signal thread to stop looping
    main_thread_->wait();                 // wait for thread cleanup to finish
}

/**
 * @brief Standard constructor.
 */
MainThread::MainThread(std::shared_ptr<LibraHebi> libra_arm,
                       std::shared_ptr<QSerialPort> ser_water,
                       std::shared_ptr<Serial> ser_servo)
    : libra_arm_(libra_arm), ser_water_(ser_water), ser_servo_(ser_servo) {}

/**
 * @brief Standard destructor.
 */
MainThread::~MainThread() {
    // Ensure data is flushed to log files and close them
    continuous_log_.close();
    snapshot_log_.close();
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {

/**
 * @brief Provides a filename-safe string of the current date and time.
 *
 * @return std::string Formatted as "yyyy-MM-ddTHH-mm-ss"
 */
std::string GetDateTimeStr() {
    auto dts = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    dts.replace(":", "-");         // replace colons (invalid in filenames)
    dts = dts.section('.', 0, 0);  // remove milliseconds
    return dts.toStdString();
}

/**
 * @brief Provides an Excel-friendly string of the current time.
 *
 * @return std::string Formatted as "HH-mm-ss.zzz"
 */
std::string GetTimestampStr() {
    return QDateTime::currentDateTime().toString("HH:mm:ss.zzz").toStdString();
}

}  // namespace

//------------------------------------------------------------------------------
// !Worker Threads
//------------------------------------------------------------------------------

/**
 * @brief Initializes a logfile with named columns.
 *
 * @param name The identifier to be assigned to the log filename.
 */
std::ofstream MainThread::InitializeLog(std::string name) {
    // Check if the log directory hasn't been created yet
    if (std::filesystem::create_directory("log")) {
        qDebug() << "[INFO] Created log directory";
    }

    // Ensure log directory exists (false returned, no error)
    assert(!std::filesystem::create_directory("log"));

    // Create log file and populate column headers
    std::ofstream logfile;
    logfile.open("log/" + GetDateTimeStr() + "_" + name + ".csv");
    logfile
        << "Time,,"
        << "TP_Roll (deg),TP_Pitch (deg),TP_J1 (deg),TP_J2 (deg),TP_J3 (deg),,"
        << "PP_Roll (deg),PP_Pitch (deg),PP_J1 (deg),PP_J2 (deg),PP_J3 (deg),,"
        << "PT_Roll (Nm),PT_Pitch (Nm),PT_J1 (Nm),PT_J2 (Nm),PT_J3 (Nm),,"
        << "A_IN,B_IN,A_OUT,B_OUT,,"
        << "TP_CamBase (deg),TP_CamPan (deg),TP_CamTilt (deg),,"
        << "Voltage (V),Current (A)" << std::endl;

    return logfile;
}

/**
 * @brief TODO: Documentation.
 *
 * @param input ...
 */
void MainThread::UpdateArmTarget(const std::array<double, 5>& input) {
    arm_target_ = input;

    if (debug_mode_) {
        qDebug() << "[DEBUG] Received the following arm targets:" << input.at(0)
                 << ", " << input.at(1) << ", " << input.at(2) << ", "
                 << input.at(3) << ", " << input.at(4);
    }
}

/**
 * @brief TODO: Rewrite.
 *        Primary request and response loop for the LIBRA app.
 *        Continuously retrieves actuator and Arduino statuses.
 *        Executed when `start()` is called on the thread.
 */
void MainThread::run() {
    // Initialize fluid system runtime variables
    int count = 0;
    uint8_t water_cmd = 0;

    // Initialize logging
    continuous_log_ = InitializeLog("continuous_log");
    snapshot_log_ = InitializeLog("shot_log");

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        /* ----- CHECK FOR COMPONENT OBJECTS (TEMPORARY) ----- */

        if (libra_arm_ == nullptr) {
            qDebug() << "[WARN] HEBI actuators are not connected! Sleeping...";
            QThread::sleep(5);  // check again in 5 seconds
            continue;
        } else if (ser_water_ == nullptr) {
            qDebug()
                << "[WARN] SerialWater Arduino is not connected! Sleeping...";
            QThread::sleep(5);  // check again in 5 seconds
            continue;
        } else if (ser_servo_ == nullptr) {
            qDebug()
                << "[WARN] SerialServo Arduino is not connected! Sleeping...";
            QThread::sleep(5);  // check again in 5 seconds
            continue;
        }

        /* ----- START ----- */

        // Retrieve HEBI actuator data
        for (auto i = 0; i < kHebiNodeCount; i++) {
            auto joint = static_cast<LibraHebi::Joint>(i);
            arm_current_.at(i).at(0) = libra_arm_->GetCommandPosition(joint);
            arm_current_.at(i).at(1) = libra_arm_->GetFeedbackPosition(joint);
            arm_current_.at(i).at(2) = libra_arm_->GetFeedbackEffort(joint);
        }

        // TODO: Center of mass visualization
        // clang-format off
        /*
        const int c_x = 3100;
        const int c_y = kWindowH / 2 + 350;

        DrawLineAA(c_x + 40 * 0, c_y - 40 * 10, c_x + 40 * (-10), c_y - 40 * 0,
                   main_color, 2.5);
        DrawLineAA(c_x + 40 * -10, c_y - 40 * 0, c_x + 40 * (0), c_y - 40 * -10,
                   main_color, 2.5);
        DrawLineAA(c_x + 40 * 0, c_y - 40 * -10, c_x + 40 * (10), c_y - 40 * 0,
                   main_color, 2.5);
        DrawLineAA(c_x + 40 * 10, c_y - 40 * 0, c_x + 40 * (0), c_y - 40 * 10,
                   main_color, 2.5);

        DrawLineAA(c_x + 40 * (0), c_y - 40 * (5), c_x + 40 * (-5),
                   c_y - 40 * (0), main_color, 2.5);
        DrawLineAA(c_x + 40 * (-5), c_y - 40 * (0), c_x + 40 * (0),
                   c_y - 40 * (-5), main_color, 2.5);
        DrawLineAA(c_x + 40 * (0), c_y - 40 * (-5), c_x + 40 * (5),
                   c_y - 40 * (0), main_color, 2.5);
        DrawLineAA(c_x + 40 * (5), c_y - 40 * (0), c_x + 40 * (0),
                   c_y - 40 * (5), main_color, 2.5);

        DrawLineAA(c_x - 500, c_y, c_x + 500, c_y, main_color, 2.5);
        DrawLineAA(c_x + 500 * cos(M_PI * 1 / 8), c_y + 500 * sin(M_PI * 1 / 8),
                   c_x - 500 * cos(M_PI * 1 / 8), c_y - 500 * sin(M_PI * 1 / 8),
                   main_color, 1);
        DrawLineAA(c_x + 500 * cos(M_PI * 3 / 8), c_y + 500 * sin(M_PI * 3 / 8),
                   c_x - 500 * cos(M_PI * 3 / 8), c_y - 500 * sin(M_PI * 3 / 8),
                   main_color, 1);
        DrawLineAA(c_x + 500 * cos(M_PI * 5 / 8), c_y + 500 * sin(M_PI * 5 / 8),
                   c_x - 500 * cos(M_PI * 5 / 8), c_y - 500 * sin(M_PI * 5 / 8),
                   main_color, 1);
        DrawLineAA(c_x + 500 * cos(M_PI * 7 / 8), c_y + 500 * sin(M_PI * 7 / 8),
                   c_x - 500 * cos(M_PI * 7 / 8), c_y - 500 * sin(M_PI * 7 / 8),
                   main_color, 1);
        DrawLineAA(c_x, c_y - 500, c_x, c_y + 500, main_color, 2.5);
        DrawTriangleAA(c_x + 500, c_y, c_x + 480, c_y + 10, c_x + 480, c_y - 10,
                       main_color, TRUE);
        DrawTriangleAA(c_x, c_y - 500, c_x + 10, c_y - 480, c_x - 10, c_y - 480,
                       main_color, TRUE);
        DrawCircleAA(c_x + arm_current_.at(0).at(2) * 40,
                     c_y - arm_current_.at(1).at(2) * 40, 15, 20, GetColor(0, 0, 0),
                     TRUE);
        DrawFormatStringToHandle(c_x + 525, c_y - 25, main_color, main_font,
                                 "Roll (Nm)");
        DrawFormatStringToHandle(c_x - 100, c_y - 550 - 25, main_color,
                                 main_font, "Pitch (Nm)");
        */
        // clang-format on

        /* ----- CAMERA ----- */

        // Calculate pitch angle to negate J3
        const double j3_pos = libra_arm_->GetCommandPosition(
            LibraHebi::Joint::kJ3);
        if (j3_pos <= 30) {
            camera_pos_.at(0) = (j3_pos <= 0) ? -j3_pos : 0;
        } else {
            camera_pos_.at(0) = 180 - j3_pos;
        }

        // Retrieve desired camera pan angle
        if (camera_dir_.at(1) != 0) {
            camera_pos_.at(1) += camera_dir_.at(1) * 90.0 / (60.0 * 60.0);
            if ((camera_pos_.at(1) > camera_setpos_.at(1))
                == (camera_dir_.at(1) == 1)) {
                camera_pos_.at(1) = camera_setpos_.at(1);
                camera_dir_.at(1) = 0;
            }
        }

        // Retrieve desired camera tilt angle
        if (camera_dir_.at(2) != 0) {
            camera_pos_.at(2) += camera_dir_.at(2) * 90.0 / (60.0 * 60.0);
            if ((camera_pos_.at(2) > camera_setpos_.at(2))
                == (camera_dir_.at(2) == 1)) {
                camera_pos_.at(2) = camera_setpos_.at(2);
                camera_dir_.at(2) = 0;
            }
        }

        // Send commands to SerialServo Arduino
        std::stringstream servo_str;
        servo_str << camera_pos_.at(0) << " " << camera_pos_.at(1) << " "
                  << camera_pos_.at(2) << "\n";
        ser_servo_->WriteStr(servo_str.str());  // send command

        /* ----- ARM and FLUID SYSTEM ----- */

        // Send commands to SerialWater Arduino
        switch (water_mode_) {
            case kStandby:  // Normal operational mode
                water_cmd = 0;

                // Excess torque (> 5.0 Nm)
                if (water_en_  // TODO: refactor this
                    && (abs(libra_arm_->GetFeedbackEffortMA()) > 5.0
                        || abs(libra_arm_->GetFeedbackEffortMB()) > 5.0)) {
                    // Pause arm movement
                    libra_arm_->Stop();
                    water_mode_ = WaterMode::kAdjust;
                }
                break;

            case kAdjust:  // Water level adjustment mode
                // Normal response to torque (2.5-5.0 Nm)
                if (water_en_
                    && (abs(libra_arm_->GetFeedbackEffortMA()) >= 2.5
                        || abs(libra_arm_->GetFeedbackEffortMB()) >= 2.5)) {
                    if (count == 0) {
                        const double theta =
                            atan2(libra_arm_->GetFeedbackEffort(
                                      LibraHebi::Joint::kPitch),
                                  libra_arm_->GetFeedbackEffort(
                                      LibraHebi::Joint::kRoll));

                        // Bit field "0b1234" -> 1: A_IN | 2: B_IN | 3: A_OUT | 4: B_OUT
                        if (theta > M_PI * 7 / 8
                            || -M_PI * 7 / 8 >= theta) {  // W
                            water_cmd = 0b1001;
                        } else if (theta > M_PI * 5 / 8) {  // NW
                            water_cmd = 0b0001;
                        } else if (theta > M_PI * 3 / 8) {  // N
                            water_cmd = 0b0011;
                        } else if (theta > M_PI * 1 / 8) {  // NE
                            water_cmd = 0b0010;
                        } else if (theta > -M_PI * 1 / 8) {  // E
                            water_cmd = 0b0110;
                        } else if (theta > -M_PI * 3 / 8) {  // SE
                            water_cmd = 0b0100;
                        } else if (theta > -M_PI * 5 / 8) {  // S
                            water_cmd = 0b1100;
                        } else {  // SW
                            water_cmd = 0b1000;
                        }
                    }
                }

                // If torque subsides (< 2.5 Nm)
                else if (count == 0) {
                    // Put fluid system on standby
                    water_mode_ = WaterMode::kStandby;
                    water_cmd = 0;

                    // Resume arm movement
                    libra_arm_->Move(arm_target_.at(0), arm_target_.at(1),
                                     arm_target_.at(2), arm_target_.at(3),
                                     arm_target_.at(4));
                }
                break;

            case kDrain:
                // Drain until ENABLE or DISABLE are clicked
                water_cmd = 0b0011;

                // Pause arm movement
                libra_arm_->Stop();
                break;
        }

        auto to_write = static_cast<char>(water_cmd);
        ser_water_->write(&to_write);  // send command

        // TODO: Visualize status of fluid system

        /* ----- LOGGING ----- */

        // Update continuous log
        if (count == 0) {
            // Timestamp
            continuous_log_ << GetTimestampStr() + ",,";

            // Actuator info
            for (auto i = 0; i < kHebiFeedbackCount; i++) {
                for (auto j = 0; j < kHebiNodeCount; j++) {
                    continuous_log_ << arm_current_.at(j).at(i) << ",";
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
            // TODO: use kCameraNodeCount
            continuous_log_ << camera_pos_.at(0) << "," << camera_pos_.at(1)
                            << "," << camera_pos_.at(2);

            // Flush the current line
            continuous_log_ << std::endl;
        }

        // TODO: refactor this weird attempt at update limiting
        count++;
        if (count == 30) {
            count = 0;
        }
    }
}

/**
 * @brief Sets whether or not verbose debug text is displayed
 * @param true to enable, false to disable
 */
void MainThread::SetDebugMode(bool enabled) {
    debug_mode_ = enabled;
}

//------------------------------------------------------------------------------
// !Menu Bar
//------------------------------------------------------------------------------

void MainWindow::on_a_debug_mode_toggled(bool checked) {
    debug_mode_ = checked;

    if (debug_mode_) {
        qDebug() << "[DEBUG] Debug mode enabled";
    }

    // Propagate to all children
    if (main_thread_ != nullptr) {
        main_thread_->SetDebugMode(debug_mode_);
    }
    if (libra_arm_ != nullptr) {
        libra_arm_->SetDebugMode(debug_mode_);
    }
    if (ser_water_ != nullptr) {
        // TODO: make new class inheriting from QSerialPort
        // ser_water_->SetDebugMode(debug_mode_);
    }
    if (ser_servo_ != nullptr) {
        ser_servo_->SetDebugMode(debug_mode_);
    }
}

/**
 * @brief Event handler for "EPOS" menu bar action "Connect".
 *        Brings up a dialog box similar to `VCS_OpenDeviceDlg()`.
 */
void MainWindow::on_a_epos_connect_triggered() {
    qDebug() << "[WARN] EPOS not yet implemented!";
    return;

    // TODO: implementation

    // Reflect changes in UI
    ui_->a_epos_connect->setEnabled(false);
    ui_->a_epos_disconnect->setEnabled(true);
    // TODO: enable relevant MainWindow buttons
}

/**
 * @brief Event handler for "EPOS" menu bar action "Disonnect".
 *        Terminates the active EPOS controller connection, if any.
 */
void MainWindow::on_a_epos_disconnect_triggered() {
    qDebug() << "[WARN] EPOS not yet implemented!";
    return;

    // TODO: implementation

    // Reflect changes in UI
    ui_->a_epos_connect->setEnabled(true);
    ui_->a_epos_disconnect->setEnabled(false);
    // TODO: disable relevant MainWindow buttons
}

/**
 * @brief Event handler for "HEBI" menu bar action "Connect".
 */
void MainWindow::on_a_hebi_connect_triggered() {
    // Only continue if "Connect" was successful
    libra_arm_ = std::make_shared<LibraHebi>();
    if (!libra_arm_->Connect()) {
        libra_arm_.reset();
        return;
    }

    if (debug_mode_) {
        qDebug() << "[DEBUG] HEBI connect returned successfully";
    }

    // Reflect changes in UI
    ui_->a_hebi_connect->setEnabled(false);
    ui_->a_hebi_disconnect->setEnabled(true);
    ui_->pb_arm_convert->setEnabled(true);
    ui_->pb_arm_start->setEnabled(true);
    ui_->pb_arm_stop->setEnabled(true);
}

/**
 * @brief Event handler for "HEBI" menu bar action "Disconnect".
 *        Terminates all active HEBI actuator connections, if any.
 */
void MainWindow::on_a_hebi_disconnect_triggered() {
    // Clear the LibraHebi object
    libra_arm_.reset();

    // Reflect changes in UI
    ui_->a_hebi_connect->setEnabled(true);
    ui_->a_hebi_disconnect->setEnabled(false);
    ui_->pb_arm_convert->setEnabled(false);
    ui_->pb_arm_start->setEnabled(false);
    ui_->pb_arm_stop->setEnabled(false);
}

/**
 * @brief Event handler for "Pumps" menu bar action "Connect".
 *        Brings up a dialog box of available serial USB devices.
 */
void MainWindow::on_a_pumps_connect_triggered() {
    // Check if SerialWater Arduino (COM3) is connected
    bool found = false;
    foreach (const QSerialPortInfo& info, QSerialPortInfo::availablePorts()) {
        if (info.portName() == "COM3") {
            found = true;
            break;
        }
        if (debug_mode_) {
            // Enumerate available serial ports
            qDebug() << "[DEBUG] Found SerialPort with following metadata";
            qDebug() << "  Port: " << info.portName();
            qDebug() << "  Description: " << info.description();
            qDebug() << "  Manufacturer: " << info.manufacturer() << "\n";
            return;
        }
    }

    if (!found) {
        qDebug() << "[ERROR] SerialWater (COM3) not found!";
        return;
    }

    // Set port options
    ser_water_->setPortName("COM3");
    ser_water_->setBaudRate(QSerialPort::Baud115200);
    ser_water_->setDataBits(QSerialPort::Data8);
    ser_water_->setParity(QSerialPort::NoParity);
    ser_water_->setStopBits(QSerialPort::OneStop);
    ser_water_->setFlowControl(QSerialPort::NoFlowControl);

    // Only continue if "open" was successful
    if (!ser_water_->open(QIODevice::ReadOnly)) {
        qDebug() << "[ERROR] Failed to open port: COM3!";
        ser_water_.reset();
        return;
    }

    // Ensure data gets processed when it's made available
    connect(ser_water_.get(), &QSerialPort::readyRead, this,
            &MainWindow::UpdatePumpVals);

    // TODO: delete this if the above Qt class works fine
#if false
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
    ser_water_ = std::make_shared<Serial>("SerialWater",
                                          "/dev/" + w_serial.GetDeviceName());
#endif

    // Reflect changes in UI
    ui_->a_pumps_connect->setEnabled(false);
    ui_->a_pumps_disconnect->setEnabled(true);
    ui_->pb_pumps_enable->setEnabled(true);
    ui_->pb_pumps_disable->setEnabled(true);
    ui_->pb_pumps_drain->setEnabled(true);
}

/**
 * @brief Event handler for "Pumps" menu bar action "Disconnect".
 *        Terminates the connection to the `SerialWater` Arduino, if it exists.
 */
void MainWindow::on_a_pumps_disconnect_triggered() {
    // Clear the Serial object
    ser_water_.reset();

    // Reflect changes in UI
    ui_->a_pumps_connect->setEnabled(true);
    ui_->a_pumps_disconnect->setEnabled(false);
    ui_->pb_pumps_enable->setEnabled(false);
    ui_->pb_pumps_disable->setEnabled(false);
    ui_->pb_pumps_drain->setEnabled(false);
}

void MainWindow::on_cb_camera_id_currentTextChanged(const QString& sel) {
    if (camera_manager_ != nullptr) {
        if (debug_mode_) {
            qDebug() << "[DEBUG] Resetting existing camera manager";
        }
        camera_manager_.reset();
    }

    // Show human-readable camera name
    ui_->l_camera_name->setText(available_cameras_[sel]);

    // Open a connection to the camera
    camera_manager_ = std::make_unique<CameraManager>(sel,
                                                      ui_->vw_camera_viewfinder,
                                                      this);
    camera_manager_->Start();

    // Reflect changes in UI
    ui_->pb_camera_capture->setEnabled(true);
    ui_->pb_camera_record->setEnabled(true);
}

/**
 * @brief Event handler for "Camera" menu bar action "Connect (servos)".
 *        Brings up a dialog box of available serial USB devices.
 */
void MainWindow::on_a_camera_servos_connect_triggered() {
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
    ser_servo_ = std::make_shared<Serial>("SerialServo",
                                          "/dev/" + w_serial.GetDeviceName());

    // Reflect changes in UI
    ui_->a_camera_servos_connect->setEnabled(false);
    ui_->a_camera_servos_disconnect->setEnabled(true);
    ui_->pb_camera_slow->setEnabled(true);
    ui_->pb_camera_fast->setEnabled(true);
}

/**
 * @brief Event handler for "Camera" menu bar action "Disconnect".
 *        Terminates the connection to the `SerialServo` Arduino, if it exists.
 */
void MainWindow::on_a_camera_servos_disconnect_triggered() {
    // Clear the Serial and CameraManager objects
    ser_servo_.reset();
    camera_manager_.reset();

    // Reflect changes in UI
    ui_->a_camera_servos_connect->setEnabled(true);
    ui_->a_camera_servos_disconnect->setEnabled(false);
    ui_->pb_camera_slow->setEnabled(false);
    ui_->pb_camera_fast->setEnabled(false);
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
    SerialDialog w_serial("ACM", debug_mode_, this);
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
    lidar_ = std::make_shared<LibraLidar>(w_serial.GetDeviceName());
}

/**
 * @brief Event handler for "LIDAR" menu bar action "Disconnect".
 *        Terminates the connection to the LIDAR, if it exists.
 */
void MainWindow::on_a_lidar_disconnect_triggered() {
    // Clear the LibraLidar object
    lidar_.reset();

    // Reflect changes in UI
    ui_->a_lidar_disconnect->setEnabled(false);
    ui_->a_lidar_about->setEnabled(false);
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
// !Arm
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_start_clicked() {
    if (libra_arm_ == nullptr) {
        qDebug() << "[WARN] HEBI actuators not connected!";
        return;
    }

    std::array<double, 5> target{0};
    target.at(0) = ui_->sb_arm_roll->text().toDouble();
    target.at(1) = ui_->sb_arm_pitch->text().toDouble();
    target.at(2) = ui_->sb_arm_j1->text().toDouble();
    target.at(3) = ui_->sb_arm_j2->text().toDouble();
    target.at(4) = ui_->sb_arm_j3->text().toDouble();

    emit CommandArm(target);
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_stop_clicked() {
    if (libra_arm_ == nullptr) {
        qDebug() << "[WARN] HEBI actuators not connected!";
        return;
    }

    // TODO: adapt code
    /*
    libra_arm_->Stop();
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_arm_convert_clicked() {
    if (libra_arm_ == nullptr) {
        qDebug() << "[WARN] HEBI actuators not connected!";
        return;
    }

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
    qDebug() << "[WARN] Not yet reimplemented!";
    return;

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
    qDebug() << "[WARN] Not yet reimplemented!";
    return;

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
    qDebug() << "[WARN] Not yet reimplemented!";
    return;

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
    qDebug() << "[WARN] Not yet reimplemented!";
    return;

    // TODO: adapt code
    /*
    ibox_theta_->SetNum(ibox_theta_->GetNum() - ibox_increment_->GetNum()
                        / ibox_r_->GetNum() * 180 / M_PI);
    OnClick(btn_convert_);
    */
}

//------------------------------------------------------------------------------
// !Counterweight
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_pumps_enable_clicked() {
    if (!ser_water_->isOpen()) {
        qDebug() << "[WARN] SerialWater not connected!";
        return;
    }

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
    if (!ser_water_->isOpen()) {
        qDebug() << "[WARN] SerialWater not connected!";
        return;
    }

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
    if (!ser_water_->isOpen()) {
        qDebug() << "[WARN] SerialWater not connected!";
        return;
    }

    // TODO: adapt code
    /*
    water_en_ = true;
    water_mode_ = WaterMode::kDrain;
    */
}

/**
 * @brief TODO: description
 */
void MainWindow::UpdatePumpVals() {
    auto data = ser_water_->readAll();
    if (debug_mode_) {
        qDebug() << "[DEBUG] Received data from SerialWater:" << data;
    }

    // TODO: implementation
}

//------------------------------------------------------------------------------
// !Camera
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_camera_slow_clicked() {
    if (ser_servo_ == nullptr) {
        qDebug() << "[WARN] SerialServo not connected!";
        return;
    }

    // TODO: adapt code
    /*
    camera_setpos_.at(1) = ibox_camera_pan_->GetNum();
    camera_setpos_.at(2) = ibox_camera_tilt_->GetNum();
    camera_dir_.at(1) = (camera_setpos_.at(1) > camera_pos_.at(1)) ? 1 :
                        ((camera_setpos_.at(1) < camera_pos_.at(1)) ? -1 : 0);
    camera_dir_.at(2) = (camera_setpos_.at(2) > camera_pos_.at(2)) ? 1 :
                        ((camera_setpos_.at(2) < camera_pos_.at(2)) ? -1 : 0);
    camera_pos_.at(1) += camera_dir_.at(1) * 90.0 / (60.0 * 60.0);
    camera_pos_.at(2) += camera_dir_.at(2) * 90.0 / (60.0 * 60.0);
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_camera_fast_clicked() {
    if (ser_servo_ == nullptr) {
        qDebug() << "[WARN] SerialServo not connected!";
        return;
    }

    // TODO: adapt code
    /*
    camera_pos_.at(1) = ibox_camera_pan_->GetNum();
    camera_pos_.at(2) = ibox_camera_tilt_->GetNum();
    */
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_camera_capture_clicked() {
    camera_manager_->Capture();

    // Display a status tip at the bottom of the UI
    statusBar()->showMessage("Saved capture to img/ directory!", kInfoLifespan);
}

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_camera_record_clicked() {
    if (camera_manager_->Record()) {
        // Clearly display a "recording" state
        ui_->pb_camera_record->setStyleSheet("color: red;");
        ui_->pb_camera_record->setText("STOP");
    } else {
        // Display a status tip at the bottom of the UI
        statusBar()->showMessage("Saved recording to vid/ directory!",
                                 kInfoLifespan);

        // Revert to original state
        ui_->pb_camera_record->setStyleSheet("color: black;");
        ui_->pb_camera_record->setText("RECORD");
    }
}

//------------------------------------------------------------------------------
// !Misc.
//------------------------------------------------------------------------------

/**
 * @brief TODO: documentation
 */
void MainWindow::on_pb_logshot_clicked() {
    qDebug() << "[WARN] Not yet reimplemented!";
    return;

    // TODO: adapt code
    /*
    const std::string dts = GetTimestampStr();
    snapshot_log_ << dts << ",,";
    for (auto i = 0; i < kHebiFeedbackCount; i++) {
        for (auto j = 0; j < kHebiNodeCount; j++) {
            snapshot_log_ << arm_current_.at(j).at(i) << ",";
        }
        snapshot_log_ << ",";
    }
    snapshot_log_ << ibox_voltage_->GetNum() << ","
                  << ibox_current_->GetNum() << "\n";

    colorize::Print("Snapshot - " + dts + " | Voltage: "
                        + std::to_string(ibox_voltage_->GetNum())
                        + " V | Current: "
                        + std::to_string(ibox_current_->GetNum()) + " A\n",
                    colorize::Level::kInfo);
    */
}

//------------------------------------------------------------------------------
// !Uncategorized
//------------------------------------------------------------------------------
