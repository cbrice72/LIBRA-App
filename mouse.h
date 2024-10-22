/******************************************************************************
 * @file   mouse.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
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

    // --- Getters & Setters ---

    int GetX();
    int GetY();
    int GetKeyPressCount(int keyCode);
    int GetKeyReleaseCount(int keyCode);

    // --- Constants ---

    const static int LEFT = 0;
    const static int RIGHT = 1;
    const static int MIDDLE = 2;

  private:
    // --- Helper Functions ---

    bool IsValidCode(int keyCode);

    // --- Data Members ---

    int m_X_, m_Y_;

    const static int kButtonNum = 8;
    int key_press_count_[kButtonNum];
    int key_release_count_[kButtonNum];
};
