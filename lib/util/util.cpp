/******************************************************************************
 * @file   util.cpp
 * @brief  Namespace for convenient, general-use functions; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "util.h"

// C++ Standard Library Headers
#include <sys/stat.h>

// Other Library Headers
#include <QDateTime>  // Qt::Core
#include <QDebug>     // Qt::Core
#if LIBRA_VERSION == 2
# include "Definitions.h"  // EPOS (Maxon)
#endif

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Utilities
 */

// For use with EPOS character buffers
constexpr uint kMaxCharBufSize = 100;

//------------------------------------------------------------------------------
// !Utilities
//------------------------------------------------------------------------------

namespace util {

/**
 * @brief Checks if the app is running in Windows Subsystem for Linux (WSL2).
 */
bool IsWslEnvironment() {
    const char* wsl_path = "/run/WSL";

    struct stat buf {};

    if (stat(wsl_path, &buf) == 0 && S_ISDIR(buf.st_mode)) {
        qWarning()
            << "[WARN] You seem to be running this on WSL2. Please ensure "
               "you have properly forwarded your USB connections.";
        return true;
    }

    return false;
}

/**
 * @brief Filename-safe string of the current date and time
 *        ("yyyy-MM-ddTHH-mm-ss").
 */
std::string GetDateTimeStr() {
    auto dts = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    dts.replace(":", "-");         // replace colons (invalid in filenames)
    dts = dts.section('.', 0, 0);  // remove milliseconds
    return dts.toStdString();
}

/**
 * @brief Excel-friendly string of the current time ("HH-mm-ss.zzz").
 */
std::string GetTimestampStr() {
    return QDateTime::currentDateTime().toString("HH:mm:ss.zzz").toStdString();
}

#if LIBRA_VERSION == 2
// NOTE: EPOS system units (from Application Notes Collection, 1.7):
//         Position:      steps (quadcounts (qc) = 4 * encoder counts / rev)
//         Velocity:      rpm
//         Acceleration:  rpm/s

/**
 * @brief Formats an EPOS library error as
 *        "<FUNCTION_NAME> [NODE_ID] failed: <ERROR TEXT>".
 *
 * @param func_name Name of the EPOS function (`VCS_...`) that failed
 * @param err Error code returned by that function
 * @param node_id EPOS node the function failed for (optional)
 *
 * @note See "EPOS Command Library", Section 8 "Error Overview" for more
 *       detailed descriptions of error messages.
 */
QString GetFormattedEposErrTxt(const std::string& func_name, const uint& err,
                               const int& node_id) {
    // Start with the function name
    auto err_msg = QString::fromStdString(func_name);

    // Add the EPOS node ID, if provided
    if (node_id != 0) {
        err_msg += QString(" [%1]").arg(node_id);
    }

    // Add the actual error text
    err_msg += " failed: ";

    // NOLINTBEGIN(*-avoid-c-arrays): EPOS C API fills a char buffer
    if (err != 0) {
        char err_text[kMaxCharBufSize];
        VCS_GetErrorInfo(err, err_text, kMaxCharBufSize);
        err_msg += QString::fromLocal8Bit(err_text);
    } else {
        // Although err = 0 means "No error" in the EPOS library, getting here
        // means the node is not connected, so replace the error text
        err_msg += QString::fromStdString("Not connected");
    }
    // NOLINTEND(*-avoid-c-arrays)

    return err_msg;
}
#endif

}  // namespace util
