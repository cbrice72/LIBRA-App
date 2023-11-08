/******************************************************************************
 * @file   main.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto
 * @date   2022/7/20
 ******************************************************************************/

// Related Header
//   (none)
// C++ Standard Library Headers
//   (none)
// Other Libraries' Headers
//   DirectX Wrapper
#include <DxLib.h>
// Project Headers
#include "AppMgr.h"

/**
 * @brief TODO.
 *
 * @return int TODO
 */
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    AppMgr mgr;
    mgr.Main();
    return 0;
}
