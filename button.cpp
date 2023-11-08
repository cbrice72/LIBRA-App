/******************************************************************************
 * @file   button.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   2022/7/5
 ******************************************************************************/

// Related Header
#include "button.h"
// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   DirectX Wrapper
#include <DxLib.h>
// Project Headers
#include "mouse.h"

/* --- TABLE OF CONTENTS ---
 * !General Functions
 */

//------------------------------------------------------------------------------
// !General Functions
//------------------------------------------------------------------------------

/**
 * @brief Constructs a new Button object.
 *
 * @param x TODO
 * @param y TODO
 * @param w TODO
 * @param h TODO
 * @param str TODO
 * @param listener TODO
 */
Button::Button(int x, int y, int w, int h, const char* str,
               OnClickListener* listener) :
    m_X_(x),
    m_Y_(y), m_W_(w), m_H_(h), m_str_(str), m_listener_(listener),
    is_pressed_(false), is_mouseover_(false),
    font_(CreateFontToHandle("Yu Gothic UI", 50, 6, DX_FONTTYPE_ANTIALIASING)) {
}

/**
 * @brief TODO.
 */
void Button::Update() {
    int x = Mouse::Instance()->GetX();
    int y = Mouse::Instance()->GetY();

    if (m_X_ <= x && x <= m_X_ + m_W_ && m_Y_ <= y && y <= m_Y_ + m_H_) {
        is_mouseover_ = true;
        if (Mouse::Instance()->GetKeyPressCount(Mouse::LEFT) == 0) {
            is_pressed_ = false;
        }
        if (Mouse::Instance()->GetKeyPressCount(Mouse::LEFT) == 1) {
            m_listener_->OnClick(this);
            is_pressed_ = true;
        }
    } else {
        is_mouseover_ = false;
        is_pressed_ = false;
    }
}

/**
 * @brief TODO.
 */
void Button::Draw() {
    int sub = 0;
    unsigned int color;
    if (is_pressed_) {
        sub = 1;
    }
    if (is_mouseover_) {
        color = GetColor(100, 100, 100);
    } else {
        color = GetColor(50, 50, 50);
    }
    int strW = GetDrawStringWidthToHandle(m_str_, strlen(m_str_), font_);
    int fontsize;
    GetFontStateToHandle(NULL, &fontsize, NULL, font_);

    DrawRoundRectAA(m_X_ + sub, m_Y_ + sub, m_X_ + m_W_ - sub,
                    m_Y_ + m_H_ - sub, 20, 20, 20, color, TRUE);
    int strX = m_X_ + m_W_ / 2 - strW / 2;
    int strY = m_Y_ + m_H_ / 2 - fontsize / 2;
    DrawStringToHandle(strX, strY, m_str_, GetColor(255, 255, 255), font_);
}

/**
 * @brief TODO.
 */
void Button::UpdateDraw() {
    Update();
    Draw();
}
