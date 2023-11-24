/******************************************************************************
 * @file   pretty_print.cpp
 * @brief  Utility header file for colorizing Windows console output.
 *
 * @author Christian Brice
 * @date   2023/11/15
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   (none)
// Project Headers
//   (none)

namespace colorize {

/**
 * @brief The log level to print a message at
 */
enum Level { kInfo = 0, kWarn, kError, kPrompt, kTitle, kDefault = -1 };

void Print(const std::string& msg, Level lvl = kDefault);

}  // namespace colorize
