/******************************************************************************
 * @file   serial.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   2022/7/5
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
// POSIX/Windows Library Headers
#include <Windows.h>

// Other Libraries' Headers
//   (none)
// Project Headers
//   (none)

/**
 * @brief TODO.
 */
class Serial {
  public:
    int open(const char* port);
    int write(BYTE data);
    int writestring(std::string str);

    HANDLE mhandle;
};
