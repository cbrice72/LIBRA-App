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
    int Open(const char* port);
    int Write(BYTE data);
    int WriteStr(std::string str);

    HANDLE handle_;
};
