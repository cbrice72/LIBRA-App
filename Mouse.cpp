/******************************************************************************
 * @file   Mouse.cpp
 * @brief  TODO.
 *
 * @author Yuto Goto
 * @date   2022/1/13
 ******************************************************************************/

// Related Header
#include "Mouse.h"
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
 * !...
 */

/**
 * @brief Constructs a new Mouse object.
 */
Mouse::Mouse() : mX(0), mY(0) {
    memset(mKeyPressingCount, 0, sizeof(mKeyPressingCount));
    memset(mKeyReleasingCount, 0, sizeof(mKeyReleasingCount));
}

/**
 * @brief TODO.
 *
 * @return true TODO
 * @return false TODO
 */
bool Mouse::Update() {
    int nowInput = GetMouseInput();  // 今のキーの入力状態を取得 - Get the input
                                     // state of the current key
    for (int i = 0; i < BUTTON_NUM; i++) {
        if ((nowInput >> i)
            & 0x01) {  // i番のキーが押されていたら - If key 'i' is pressed...
            if (mKeyReleasingCount[i]
                > 0) {  // 離されカウンタが0より大きければ - If the release
                        // count is > 0...
                mKeyReleasingCount[i] = 0;  // 0に戻す - Reset back to 0
            }
            mKeyPressingCount[i]++;  // 押されカウンタを増やす - Increase pushed
                                     // counter
        } else {  // i番のキーが離されていたら - If key 'i' is released...
            if (mKeyPressingCount[i] > 0) {  // 押されカウンタが0より大きければ
                                             // - If the push count is > 0...
                mKeyPressingCount[i] = 0;  // 0に戻す - Reset back to 0
            }
            mKeyReleasingCount[i]++;  // 離されカウンタを増やす - Increase
                                      // released counter
        }
    }
    GetMousePoint(&mX, &mY);
    return true;
}

/**
 * @brief Gets the number of frames in which the `keyCode` key is pressed.
 *
 * @param keyCode TODO
 * @return int TODO
 *
 * @note `keyCode`のキーが押されているフレーム数を返す。
 */
int Mouse::GetPressingCount(int keyCode) {
    if (!Mouse::IsAvailableCode(keyCode)) {
        return -1;
    }
    return mKeyPressingCount[keyCode];
}

/**
 * @brief Gets the number of frames in which the `keyCode` key is depressed.
 *
 * @param keyCode TODO
 * @return int TODO
 *
 * @note `keyCode`のキーが離されているフレーム数を返す。
 */
int Mouse::GetReleasingCount(int keyCode) {
    if (!Mouse::IsAvailableCode(keyCode)) {
        return -1;
    }
    return mKeyReleasingCount[keyCode];
}

/**
 * @brief Checks if `keyCode` is a valid key.
 *
 * @param keyCode TODO
 * @return true TODO
 * @return false TODO
 *
 * @note `keyCode`が有効な値かチェックする。
 */
bool Mouse::IsAvailableCode(int keyCode) {
    if (0 <= keyCode && keyCode < BUTTON_NUM) {
        return true;
    }
    return false;
}

/**
 * @brief TODO.
 *
 * @return int TODO
 */
int Mouse::GetX() {
    return mX;
}

/**
 * @brief TODO.
 *
 * @return int TODO
 */
int Mouse::GetY() {
    return mY;
}
