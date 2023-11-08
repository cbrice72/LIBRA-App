/******************************************************************************
 * @file   main.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
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
#include "app_mgr.h"

/**
 * @brief The application entry point.
 *
 * @return int Exit value of the function.
 */
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    AppMgr mgr;
    mgr.Main();
    return 0;
}
