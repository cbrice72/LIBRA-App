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

    // --- Getters & Setters ---

    float GetNum();
    void SetNum(float num);

  private:
    // --- Data Members ---
    int m_X_, m_Y_, m_W_, m_H_;

    bool is_pressed_;
    bool is_mouseover_;

    int font_;
    int input_handle_;
    int main_color_;

    int count_ = 0;
};
