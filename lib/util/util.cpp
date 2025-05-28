/******************************************************************************
 * @file   util.cpp
 * @brief  Namespace for convenient, general-use functions; implementation file.
 *
 * @author brice.c.aa
 ******************************************************************************/

// Related Header
#include "util.h"

// C++ Standard Library Headers
//   (none)

// Other Library Headers
#include "Definitions.h"  // EPOS (Maxon)
#include <QDateTime>      // Qt::Core
#include <QDebug>         // Qt::Core
#ifdef BUILD_WITH_ROS2
# include <rclcpp/rclcpp.hpp>  // ROS2 Core
#endif

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Local Helpers
 * !Utilities
 */

//------------------------------------------------------------------------------
// !Local Helpers
//------------------------------------------------------------------------------

// For use with EPOS character buffers
constexpr uint kMaxCharBufSize = 100;

/**
 * @brief Convenience wrapper around `VCS_GetErrorInfo()`.
 *
 * @param err The Maxon error code
 * @return Error text corresponding to the provided error code
 *
 * @note This function is outside the `util` namespace because it is simply a
 *       helper function for `PrintEPOSErr()`; it shouldn't be necessary to use
 *       it outside of this file.
 *
 * @see "EPOS Command Library", Section 8 "Error Overview" for more detailed
 *      descriptions of error messages.
 */
std::string GetMaxonErrTxt(uint err) {
    if (err != 0) {
        char err_text[kMaxCharBufSize];
        VCS_GetErrorInfo(err, err_text, kMaxCharBufSize);
        return std::string(err_text);
    } else {
        // Although err = 0 means "No error" in the EPOS library, if we are here
        // it means the node is not connected, so replace the error text.
        return "Not connected";
    }
}

//------------------------------------------------------------------------------
// !Utilities
//------------------------------------------------------------------------------

namespace util {

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

/**
 * Note concerning EPOS system units (from Application Notes Collection, 1.7)
 *
 * Position Units:  steps (quadcounts (qc) = 4 * encoder counts / revolution)
 * Velocity:        rpm
 * Acceleration:    rpm/s
 */

/**
 * @brief Formats an error message for EPOS library functions and prints it.
 *        Supports both C++ `stderr` and Qt's `stderr`-like message handler.
 *
 * @param func_name The name of the EPOS function (`VCS_...`) that failed
 * @param err The error code returned by said function
 * @param node_id [Optional] The EPOS node for which the function failed
 */
void PrintEPOSErr(std::string func_name, uint err, int node_id) {
#if defined(QT_VERSION)  // Qt-compatible code
    if (node_id == 0) {
        qCritical() << func_name.c_str()
                    << "failed:" << QString::fromStdString(GetMaxonErrTxt(err));
    } else {
        qCritical() << func_name.c_str() << "[" << node_id << "] failed:"
                    << QString::fromStdString(GetMaxonErrTxt(err));
    }
#elif  // general-purpose code
    if (node_id == 0) {
        std::cerr << func_name.c_str() << "failed:" << GetMaxonErrTxt(err);
    } else {
        std::cerr << func_name.c_str() << "[" << node_id
                  << "] failed:" << GetMaxonErrTxt(err);
    }
#endif
}

}  // namespace util
