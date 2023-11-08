/******************************************************************************
 * @file   mouse.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   2022/1/13
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   (none)
// Project Headers
#include "singleton.h"

/**
 * @brief TODO.
 */
class Mouse : public Singleton<Mouse> {
  public:
    Mouse();

    friend Singleton<Mouse>;

    bool Update();
    int GetPressingCount(int keyCode);
    int GetReleasingCount(int keyCode);
    int GetX();
    int GetY();

    const static int LEFT = 0;
    const static int RIGHT = 1;
    const static int MIDDLE = 2;

  private:
    bool IsAvailableCode(int keyCode);

    const static int BUTTON_NUM = 8;
    int mKeyPressingCount[BUTTON_NUM];
    int mKeyReleasingCount[BUTTON_NUM];
    int mX, mY;
};
