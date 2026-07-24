/******************************************************************************
 * @file   util.h
 * @brief  Namespace for convenient, general-use functions; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Other Library Headers
#include <QString>  // Qt::Core

// Project Headers
//   (none)

namespace util {

bool IsWslEnvironment();

std::string GetDateTimeStr();
std::string GetTimestampStr();

#if LIBRA_VERSION == 2
QString GetFormattedEposErrTxt(const std::string& func_name, const uint& err,
                               const int& node_id = 0);
#endif

}  // namespace util
