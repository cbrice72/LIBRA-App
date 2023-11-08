/******************************************************************************
 * @file   mouse.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
 * @date   2022/1/13
 ******************************************************************************/

// Related Header
#include "mouse.h"
// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   DirectX Wrapper
#include <DxLib.h>

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !General Functions
 * !Getters & Setters
 * !Helper Functions
 */

//------------------------------------------------------------------------------
// !General Functions
//------------------------------------------------------------------------------

/**
 * @brief Constructs a new Mouse object.
 */
Mouse::Mouse() : m_X_(0), m_Y_(0) {
    memset(key_press_count_, 0, sizeof(key_press_count_));
    memset(key_release_count_, 0, sizeof(key_release_count_));
}

/**
 * @brief TODO.
 *
 * @return true TODO
 * @return false TODO
 */
bool Mouse::Update() {
    int nowInput = GetMouseInput();  // 今のキーの入力状態を取得
                                     // Get the input_ state of the current key

    for (int i = 0; i < kButtonNum; i++) {
        // i番のキーが押されていたら - If "i" key is pressed
        if ((nowInput >> i) & 0x01) {
            // 離されカウンタが0より大きければ - If the "released" count is > 0
            if (key_release_count_[i] > 0) {
                key_release_count_[i] = 0;  // 0に戻す - Reset to 0
            }
            key_press_count_[i]++;  // 押されカウンタを増やす
                                    // Increase "pressed" counter
        } else {  // i番のキーが離されていたら - If "i" key is released
            // 押されカウンタが0より大きければ - If the "pressed" count is > 0
            if (key_press_count_[i] > 0) {
                key_press_count_[i] = 0;  // 0に戻す - Reset to 0
            }
            key_release_count_[i]++;  // 離されカウンタを増やす
                                      // Increase "released" counter
        }
    }

    GetMousePoint(&m_X_, &m_Y_);
    return true;
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 *
 * @return int TODO
 */
int Mouse::GetX() {
    return m_X_;
}

/**
 * @brief TODO.
 *
 * @return int TODO
 */
int Mouse::GetY() {
    return m_Y_;
}

/**
 * @brief Gets the number of frames in which the `keyCode` key is pressed.
 *
 * @param keyCode TODO
 * @return int TODO
 *
 * @note `keyCode`のキーが押されているフレーム数を返す。
 */
int Mouse::GetKeyPressCount(int keyCode) {
    if (!Mouse::IsValidCode(keyCode)) {
        return -1;
    }
    return key_press_count_[keyCode];
}

/**
 * @brief Gets the number of frames in which the `keyCode` key is depressed.
 *
 * @param keyCode TODO
 * @return int TODO
 *
 * @note `keyCode`のキーが離されているフレーム数を返す。
 */
int Mouse::GetKeyReleaseCount(int keyCode) {
    if (!Mouse::IsValidCode(keyCode)) {
        return -1;
    }
    return key_release_count_[keyCode];
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Checks if `keyCode` is a valid key.
 *
 * @param keyCode TODO
 * @return true TODO
 * @return false TODO
 *
 * @note `keyCode`が有効な値かチェックする。
 */
bool Mouse::IsValidCode(int keyCode) {
    return (keyCode >= 0 && keyCode < kButtonNum);
}
