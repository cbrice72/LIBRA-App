/******************************************************************************
 * @file   .cpp
 * @brief  TODO
 *
 * @author Yuto Goto
 * @date   2022/7/5
 ******************************************************************************/

#pragma once

// Related Header
//   (none)
// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   (none)
// Project Headers
#include "OnClickListener.h"
#include "View.h"

class Button : public View {
  public:
    Button(int x, int y, int w, int h, const char* str,
           OnClickListener* listener);
    void Update() override;
    void Draw() override;
    void UpdateDraw();

  private:
    int mX, mY, mW, mH;
    const char* mStr;
    OnClickListener* mListener;

    bool IsPressed;
    bool IsMouseover;

    int font;
};
