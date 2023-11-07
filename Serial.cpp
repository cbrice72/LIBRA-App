/******************************************************************************
 * @file   .cpp
 * @brief  TODO
 *
 * @author Yuto Goto
 * @date   2022/7/5
 ******************************************************************************/

// Related Header
#include "Serial.h"
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
 * !...
 */

int Serial::open(const char* port) {
    DWORD dwErrorMask;
    COMSTAT comStat;
    DWORD dwCount;
    bool Ret = 0;
    char str[30];
    char msstr[100];

    // Arduinoの通信準備 - Preparing Arduino for Communication

    // 1.ポートをオープン - Open port
    sprintf(str, "\\\\.\\%s", port);
    mhandle = CreateFile(_T(str), GENERIC_WRITE | GENERIC_READ, 0, NULL,
                         OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (mhandle == INVALID_HANDLE_VALUE) {
        sprintf(msstr,
                "ポートを開けませんでした。\nArduino(%s)を接続してください。",
                port);
        // MessageBox(NULL, TEXT(msstr), TEXT("HEBI App：通信エラー"), MB_OK |
        // MB_ICONWARNING);
        return -1;
    }

    // 2.送受信バッファ初期化 - Transmit/receive buffer initialization
    Ret = SetupComm(mhandle, 1024, 1024);
    if (!Ret) {
        MessageBox(NULL, TEXT("セットアップに失敗しました。"),
                   TEXT("HEBI App：通信エラー"), MB_OK | MB_ICONERROR);
        CloseHandle(mhandle);
        return -1;
    }

    Ret = PurgeComm(mhandle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR
                                 | PURGE_RXCLEAR);
    if (!Ret) {
        MessageBox(NULL, TEXT("初期化に失敗しました。"),
                   TEXT("HEBI App：通信エラー"), MB_OK | MB_ICONERROR);
        CloseHandle(mhandle);
        return -1;
    }

    // 3.基本通信条件の設定 - Setting basic communication conditions
    DCB dcb;
    GetCommState(mhandle, &dcb);
    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = 115200;
    dcb.fBinary = TRUE;
    dcb.ByteSize = 8;
    dcb.fParity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    Ret = SetCommState(mhandle, &dcb);
    if (!Ret) {
        MessageBox(NULL, TEXT("基本通信条件の設定に失敗しました。"),
                   TEXT("HEBI App：通信エラー"), MB_OK | MB_ICONERROR);
        CloseHandle(mhandle);
        return -1;
    }

    // 4.受信 - Reception
    ClearCommError(mhandle, &dwErrorMask, &comStat);
    dwCount = comStat.cbInQue;

    return 0;
}

int Serial::write(BYTE data) {
    DWORD dwSendSize;
    if (WriteFile(mhandle, &data, sizeof(data), &dwSendSize, NULL) == 0) {
        return -1;
    }
    return 0;
}

int Serial::writestring(std::string str) {
    for (int i = 0; i < (int)str.size(); i++) {
        if (write(str[i])) {
            return -1;
        }
    }

    return 0;
}
