/******************************************************************************
 * @file   serial.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   2022/7/5
 ******************************************************************************/

// Related Header
#include "serial.h"
// C++ Standard Library Headers
#include <cstdio>
#include <cstdlib>
// POSIX/Windows Library Headers
#include <Windows.h>
#include <tchar.h>

// Other Libraries' Headers
//   (none)
// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !General Functions
 */

//------------------------------------------------------------------------------
// !General Functions
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 *
 * @param port TODO
 * @return int TODO
 */
int Serial::Open(const char* port) {
    DWORD dwErrorMask;
    COMSTAT comStat;
    DWORD dwCount;
    BOOL retval = 0;
    char str[30];
    char msstr[100];

    // Arduinoの通信準備 - Preparing Arduino for Communication

    // 1. ポートをオープン - Open port
    sprintf(str, "\\\\.\\%s", port);
    handle_ = CreateFile(_T(str), GENERIC_WRITE | GENERIC_READ, 0, NULL,
                         OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (handle_ == INVALID_HANDLE_VALUE) {
        sprintf(msstr,
                "ポートを開けませんでした。\nArduino(%s)を接続してください。",
                port);
        // MessageBox(NULL, TEXT(msstr), TEXT("HEBI App：通信エラー"), MB_OK |
        // MB_ICONWARNING);
        return -1;
    }

    // 2. 送受信バッファ初期化 - Transmit/receive buffer initialization
    retval = SetupComm(handle_, 1024, 1024);
    if (!retval) {
        MessageBox(NULL, TEXT("セットアップに失敗しました。"),
                   TEXT("HEBI App：通信エラー"), MB_OK | MB_ICONERROR);
        CloseHandle(handle_);
        return -1;
    }

    retval = PurgeComm(handle_, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR
                                    | PURGE_RXCLEAR);
    if (!retval) {
        MessageBox(NULL, TEXT("初期化に失敗しました。"),
                   TEXT("HEBI App：通信エラー"), MB_OK | MB_ICONERROR);
        CloseHandle(handle_);
        return -1;
    }

    // 3. 基本通信条件の設定 - Setting basic communication conditions
    DCB dcb;
    GetCommState(handle_, &dcb);
    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = 115200;
    dcb.fBinary = TRUE;
    dcb.ByteSize = 8;
    dcb.fParity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    retval = SetCommState(handle_, &dcb);
    if (!retval) {
        MessageBox(NULL, TEXT("基本通信条件の設定に失敗しました。"),
                   TEXT("HEBI App：通信エラー"), MB_OK | MB_ICONERROR);
        CloseHandle(handle_);
        return -1;
    }

    // 4. 受信 - Reception
    ClearCommError(handle_, &dwErrorMask, &comStat);
    dwCount = comStat.cbInQue;

    return 0;
}

/**
 * @brief TODO.
 *
 * @param data TODO
 * @return int TODO
 */
int Serial::Write(BYTE data) {
    DWORD dwSendSize;
    if (WriteFile(handle_, &data, sizeof(data), &dwSendSize, NULL) == 0) {
        return -1;
    }

    return 0;
}

/**
 * @brief TODO.
 *
 * @param str TODO
 * @return int TODO
 */
int Serial::WriteStr(std::string str) {
    for (int i = 0; i < (int)str.size(); i++) {
        if (Write(str[i])) {
            return -1;
        }
    }

    return 0;
}
