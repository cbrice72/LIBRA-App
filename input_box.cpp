/******************************************************************************
 * @file   input_box.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 ******************************************************************************/

// Related Header
#include "input_box.h"
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
 * !Getters & Setters
 */

//------------------------------------------------------------------------------
// !General Functions
//------------------------------------------------------------------------------

/**
 * @brief Constructs a new InputBox object.
 *
 * @param x TODO
 * @param y TODO
 */
InputBox::InputBox(int x, int y)
    : m_X_(x), m_Y_(y), m_W_(220), m_H_(100), is_pressed_(false),
      is_mouseover_(false),
      font_(CreateFontToHandle("Yu Gothic UI", 50, 5, DX_FONTTYPE_ANTIALIASING)),
      input_handle_(MakeKeyInput(7, FALSE, TRUE, TRUE)),
      main_color_(GetColor(50, 50, 50)) {}

void InputBox::Update() {
    int x = Mouse::Instance()->GetX();
    int y = Mouse::Instance()->GetY();

    if (m_X_ <= x && x <= m_X_ + m_W_ && m_Y_ <= y && y <= m_Y_ + m_H_) {
        is_mouseover_ = true;
        if (Mouse::Instance()->GetKeyPressCount(Mouse::LEFT) == 0) {
            is_pressed_ = false;
        }
        if (Mouse::Instance()->GetKeyPressCount(Mouse::LEFT) == 1) {
            is_pressed_ = true;
            // 作成したキー入力ハンドルをアクティブにする
            // - Activate the created key input_ handle
            SetActiveKeyInput(input_handle_);
            count_ = 0;
        }
    } else {
        is_mouseover_ = false;
        is_pressed_ = false;
    }
}

/**
 * @brief TODO.
 */
void InputBox::Draw() {
    int fontsize;
    GetFontStateToHandle(NULL, &fontsize, NULL, font_);

    // 四角形を描画 - Draw rectangle
    DrawBoxAA(m_X_, m_Y_, m_X_ + m_W_, m_Y_ + m_H_, main_color_, FALSE,
              (is_mouseover_ || GetActiveKeyInput() == input_handle_) ? 5.5
                                                                      : 2.5);

    // 入力途中の文字列を描画 - Draw string in middle of
    // input_
    // (?)
    char string[20];
    GetKeyInputString(string, input_handle_);
    int strX = m_X_ + m_W_
               - GetDrawStringWidthToHandle(string, strlen(string), font_) - 20;
    int strY = m_Y_ + m_H_ / 2 - fontsize / 2;
    DrawFormatStringToHandle(strX, strY, main_color_, font_, string);

    // カーソルを描画 - Draw cursor
    if (GetActiveKeyInput() == input_handle_) {
        int cursorX = strX
                      + GetDrawStringWidthToHandle(string,
                                                   GetKeyInputCursorPosition(
                                                       input_handle_),
                                                   font_);
        if (count_ < 30) {
            DrawLineAA(cursorX, strY + 7, cursorX, strY + 50, main_color_, 3.5);
        }
        count_++;
        if (count_ >= 60) {
            count_ = 0;
        }
    }
}

/**
 * @brief TODO.
 */
void InputBox::UpdateDraw() {
    Update();
    Draw();
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 *
 * @return float TODO
 */
float InputBox::GetNum() {
    return GetKeyInputNumberToFloat(input_handle_);
}

/**
 * @brief TODO.
 *
 * @param num TODO
 */
void InputBox::SetNum(float num) {
    char str[20];
    sprintf(str, "%.2f", num);
    SetKeyInputString(str, input_handle_);
}
