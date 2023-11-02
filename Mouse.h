#pragma once

#include "Singleton.h"

class Mouse : public Singleton<Mouse> {

    Mouse();
    friend Singleton<Mouse>;

public:
    bool Update();	// 更新 - Update
    int GetPressingCount(int keyCode);  // keyCodeのキーが押されているフレーム数を取得
                                        // Gets the number of frames which 'keyCode' is pressed
    int GetReleasingCount(int keyCode);  // keyCodeのキーが離されているフレーム数を取得
                                         // Gets the number of frames which 'keyCode' is released
    int GetX();
    int GetY();
    const static int LEFT   = 0;
    const static int RIGHT  = 1;
    const static int MIDDLE = 2;

private:
    const static int BUTTON_NUM = 8;
    int mKeyPressingCount [BUTTON_NUM];  // 押されカウンタ - Pressed counter
    int mKeyReleasingCount[BUTTON_NUM];  // 離されカウンタ - Released counter
    int mX, mY;

    bool IsAvailableCode(int keyCode);  // keyCodeが有効なキー番号か問う - Queries if 'keyCode' is valid
};
