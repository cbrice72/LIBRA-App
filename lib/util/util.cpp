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

#if LIBRA_VERSION == 2
/**
 * Note concerning EPOS system units (from Application Notes Collection, 1.7)
 *
 * Position Units:  steps (quadcounts (qc) = 4 * encoder counts / revolution)
 * Velocity:        rpm
 * Acceleration:    rpm/s
 */

/**
 * @brief Returns the following formatted message for EPOS library errors:
 *        "<FUNCTION_NAME> [NODE_ID] failed: <ERROR TEXT>"
 *
 * @param func_name The name of the EPOS function (`VCS_...`) that failed
 * @param err The error code returned by said function
 * @param node_id [Optional] The EPOS node for which the function failed
 *
 * @see "EPOS Command Library", Section 8 "Error Overview" for more detailed
 *      descriptions of error messages.
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

    // NOLINTBEGIN
    if (err != 0) {
        char err_text[kMaxCharBufSize];
        VCS_GetErrorInfo(err, err_text, kMaxCharBufSize);
        err_msg += QString::fromLocal8Bit(err_text);
    } else {
        // Although err = 0 means "No error" in the EPOS library, if we are here
        // it means the node is not connected, so replace the error text.
        err_msg += QString::fromStdString("Not connected");
    }
    // NOLINTEND

    return err_msg;
}
#endif

}  // namespace util
