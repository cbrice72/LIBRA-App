/******************************************************************************
 * @file   input_box.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   2022/7/5
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   (none)
// Project Headers
#include "onclick_listener.h"
#include "view.h"

/**
 * @brief TODO.
 */
class InputBox : public View {
  public:
    InputBox(int x, int y);

    void Update() override;
    void Draw() override;
    void UpdateDraw();

    float GetNum();
    void SetNum(float num);

  private:
    int mX, mY, mW, mH;

    bool IsPressed;
    bool IsMouseover;

    int font;
    int InputHandle;
    int maincolor;

    int count = 0;
};
