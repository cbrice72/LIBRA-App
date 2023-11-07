/******************************************************************************
 * @file   .cpp
 * @brief  TODO
 *
 * @author Yuto Goto
 * @date   2022/7/5
 ******************************************************************************/

// Related Header
#include "Button.h"
// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   DirectX Wrapper
#include <DxLib.h>
// Project Headers
#include "Mouse.h"

/* --- TABLE OF CONTENTS ---
 * !...
 */

Button::Button(int x, int y, int w, int h, const char* str,
               OnClickListener* listener) :
    mX(x),
    mY(y),
    mW(w),
    mH(h),
    mStr(str),
    mListener(listener),
    IsPressed(false),
    IsMouseover(false),
    font(CreateFontToHandle("Yu Gothic UI", 50, 6, DX_FONTTYPE_ANTIALIASING)) {}

void Button::Update() {
    int x = Mouse::Instance()->GetX();
    int y = Mouse::Instance()->GetY();

    if (mX <= x && x <= mX + mW && mY <= y && y <= mY + mH) {
        IsMouseover = true;
        if (Mouse::Instance()->GetPressingCount(Mouse::LEFT) == 0) {
            IsPressed = false;
        }
        if (Mouse::Instance()->GetPressingCount(Mouse::LEFT) == 1) {
            mListener->OnClick(this);
            IsPressed = true;
        }
    } else {
        IsMouseover = false;
        IsPressed = false;
    }
}

void Button::Draw() {
    int sub = 0;
    unsigned int color;
    if (IsPressed) {
        sub = 1;
    }
    if (IsMouseover) {
        color = GetColor(100, 100, 100);
    } else {
        color = GetColor(50, 50, 50);
    }
    int strW = GetDrawStringWidthToHandle(mStr, strlen(mStr), font);
    int fontsize;
    GetFontStateToHandle(NULL, &fontsize, NULL, font);

    DrawRoundRectAA(mX + sub, mY + sub, mX + mW - sub, mY + mH - sub, 20, 20,
                    20, color, TRUE);
    int strX = mX + mW / 2 - strW / 2;
    int strY = mY + mH / 2 - fontsize / 2;
    DrawStringToHandle(strX, strY, mStr, GetColor(255, 255, 255), font);
}

void Button::UpdateDraw() {
    Update();
    Draw();
}
