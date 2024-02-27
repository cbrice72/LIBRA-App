/******************************************************************************
 * @file   utility.h
 * @brief  Namespace for convenient, general-use functions; implementation file.
 *
 * @author brice.c.aa
 * @date   2023/7/20
 ******************************************************************************/

// Related Header
#include "utility.h"
// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   Maxon
#include "Definitions.h"
//   Qt
#include <QDebug>
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
#if defined(QT_VERSION)
    if (node_id == 0) {
        qCritical() << func_name.c_str()
                    << "failed:" << QString::fromStdString(GetMaxonErrTxt(err));
    } else {
        qCritical() << func_name.c_str() << "[" << node_id << "] failed:"
                    << QString::fromStdString(GetMaxonErrTxt(err));
    }
#elif
    if (node_id == 0) {
        std::cerr << func_name.c_str() << "failed:" << GetMaxonErrTxt(err);
    } else {
        std::cerr << func_name.c_str() << "[" << node_id
                  << "] failed:" << GetMaxonErrTxt(err);
    }
#endif
}

}  // namespace util
