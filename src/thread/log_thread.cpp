/******************************************************************************
 * @file   log_thread.h
 * @brief  Consolidated logging class for LIBRA sensor data; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "log_thread.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include <QDebug>  // Qt::Core

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Thread Overrides
 * !Log Commands (slots)
 */

/* Constants */

//   (none)

/**
 * @brief Standard constructor.
 *
 * @param parent Owning Qt widget
 * @param debug_mode Whether verbose debug text should be output
 */
LogThread::LogThread(QObject* parent, const bool& debug_mode)
    : QThread(parent), debug_mode_(debug_mode) {}

/**
 * @brief Standard destructor.
 */
LogThread::~LogThread() {
    // Ensure data is flushed to log files and close them
    if (continuous_log_.is_open()) {
        continuous_log_.close();
    }
    if (snapshot_log_.is_open()) {
        snapshot_log_.close();
    }

    if (debug_mode_) {
        qDebug() << "[DEBUG] Cleaned up LogThread";
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

namespace {  // local to this file

/**
 * @brief Provides a filename-safe string of the current date and time.
 *
 * @return String formatted as "yyyy-MM-ddTHH-mm-ss"
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
 * @return String formatted as "HH-mm-ss.zzz"
 */
std::string GetTimestampStr() {
    return QDateTime::currentDateTime().toString("HH:mm:ss.zzz").toStdString();
}

}  // namespace

/**
 * @brief Initializes a logfile with named columns.
 *
 * @param name The identifier to be assigned to the log filename.
 */
std::ofstream LogThread::InitializeLog(std::string name) {
    // Check if the log directory hasn't been created yet
    if (std::filesystem::create_directory("log")) {
        qDebug() << "[INFO] Created log directory";
    }

    // Ensure log directory exists (false returned, no error)
    assert(!std::filesystem::create_directory("log"));

    // Create log file and populate column headers
    std::ofstream logfile;
    logfile.open("log/" + GetDateTimeStr() + "_" + name + ".csv");
    logfile << "Time,,"
            << "TP_Yaw (deg),TP_Pitch (deg),,"
            << "AP_Yaw (deg),AP_Pitch (deg),,"
            << "AT_Yaw (Nm),AT_Pitch (Nm),,"
            << "PUMP_IN,PUMP_OUT,,"
            << "TP_ManipBase (deg),TP_ManipPan (deg),TP_ManipTilt (deg),,"
            << "Voltage (V),Current (A)" << std::endl;

    return logfile;
}

//------------------------------------------------------------------------------
// !Thread Overrides
//------------------------------------------------------------------------------

/**
 * @brief Main log generation loop.
 */
void LogThread::run() {
    if (debug_mode_) {
        qDebug() << "[DEBUG] Initialized LogThread";
    }

    // Initialize logging
    continuous_log_ = InitializeLog("continuous_log");
    snapshot_log_ = InitializeLog("shot_log");

    // Initialize thread variables for efficiency
    //   (none)

    // Loop until MainWindow calls QThread::requestInterruption()
    while (!isInterruptionRequested()) {
        // TODO: implementation

        // Update continuous log
        /*
        if (something) {
            // Timestamp
            continuous_log_ << GetTimestampStr() + ",,";

            // Actuator info
            for (auto i = 0; i < kHebiFeedbackCount; i++) {
                continuous_log_ << arm_current_.at(i) << ",,";
            }

            // Fluid system info
            for (auto i = 0; i < kFluidStateCount; i++) {
                continuous_log_ << ((cmd_water & (1 << (3 - i))) ? 1 : 0)
                                << ",";
            }
            continuous_log_ << ",";

            // Camera actuator info
            // TODO: use kCameraNodeCount
            continuous_log_ << manip_pos_.at(0) << "," << manip_pos_.at(1)
                            << "," << manip_pos_.at(2);

            // Flush the current line
            continuous_log_ << std::endl;
        }
        */

        QThread::msleep(100);  // update 10 times/second (theoretically)
    }
}

//------------------------------------------------------------------------------
// !Log Commands (slots)
//------------------------------------------------------------------------------

/**
 * @brief Saves a "screenshot" of the current data to a separate log file.
 */
void LogThread::TakeLogShot() {
    // TODO: implementation
}
