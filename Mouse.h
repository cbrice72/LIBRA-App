/******************************************************************************
 * @file   .h
 * @brief  TODO
 *
 * @author Yuto Goto
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
#include "Singleton.h"

class Mouse : public Singleton<Mouse> {
  public:
    Mouse();

    friend Singleton<Mouse>;

    bool Update();
    // keyCodeのキーが押されているフレーム数を取得
    // Gets the number of frames in which the keyCode key is pressed
    int GetPressingCount(int keyCode);
    // keyCodeのキーが離されているフレーム数を取得
    // Gets the number of frames in which the keyCode key is depressed
    int GetReleasingCount(int keyCode);
    int GetX();
    int GetY();

    const static int LEFT = 0;
    const static int RIGHT = 1;
    const static int MIDDLE = 2;

  private:
    // keyCodeが有効なキー番号か問う
    // Checks if keyCode is valid
    bool IsAvailableCode(int keyCode);

    const static int BUTTON_NUM = 8;
    int mKeyPressingCount[BUTTON_NUM];
    int mKeyReleasingCount[BUTTON_NUM];
    int mX, mY;
};
