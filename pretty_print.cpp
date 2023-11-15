/******************************************************************************
 * @file   pretty_print.cpp
 * @brief  Utility implementation file for colorizing Windows console output
 *
 * @author Christian Brice
 * @date   2023/11/15
 ******************************************************************************/

// Related Header
#include "pretty_print.h"
// C++ Standard Library Headers
#include <iostream>
// POSIX/Windows Library Headers
#include <Windows.h>

// Other Libraries' Headers
//   (none)
// Project Headers
//   (none)

#ifndef FOREGROUND_YELLOW
# define FOREGROUND_YELLOW (FOREGROUND_GREEN | FOREGROUND_RED)
#endif
#ifndef FOREGROUND_WHITE
# define FOREGROUND_WHITE (FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED)
#endif

namespace colorize {

/**
 * @brief Pretty-prints a given message based on log level attributes.
 *
 * @param msg Message to print to the console
 * @param lvl Log level of message (determines coloring)
 */
void Print(const std::string& msg, Level lvl) {
    std::string prefix;
    unsigned attr = 0;

    // Determine the
    switch (lvl) {
        case kInfo:
            prefix = "[INFO] ";
            attr = FOREGROUND_BLUE;
            break;
        case kWarn:
            prefix = "[WARN] ";
            attr = FOREGROUND_YELLOW;
            break;
        case kError:
            prefix = "[ERROR] ";
            attr = FOREGROUND_RED;
            break;
        case kDefault:
            // do nothing
            break;
    }

    // Retrieve handle to Windows std::out
    HANDLE h_console = GetStdHandle(STD_OUTPUT_HANDLE);

    // Pretty-print the message
    SetConsoleTextAttribute(h_console, attr | FOREGROUND_INTENSITY);
    std::cout << prefix;
    SetConsoleTextAttribute(h_console,
                            FOREGROUND_WHITE);  // reset console attributes
    std::cout << msg;
}

}  // namespace colorize
