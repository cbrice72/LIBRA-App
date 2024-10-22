/******************************************************************************
 * @file   button.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
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
#include "onclick_listener.h"
#include "view.h"

/**
 * @brief TODO.
 */
class Button : public View {
  public:
    Button(int x, int y, int w, int h, const char* str,
           OnClickListener* listener);

    void Update() override;
    void Draw() override;
    void UpdateDraw();

  private:
    // --- Data Members ---

    int m_X_, m_Y_, m_W_, m_H_;
    const char* m_str_;
    OnClickListener* m_listener_;

    bool is_pressed_;
    bool is_mouseover_;

    int font_;
};
