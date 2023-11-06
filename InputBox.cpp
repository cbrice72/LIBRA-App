#include "InputBox.h"
#include "Mouse.h"
#include <DxLib.h>

InputBox::InputBox(int x, int y) : IsPressed(false), IsMouseover(false) {
    mX = x;
    mY = y;
    mW = 220;
    mH = 100;
    font = CreateFontToHandle("Yu Gothic UI", 50, 5, DX_FONTTYPE_ANTIALIASING);
    InputHandle =
        MakeKeyInput(7, FALSE, TRUE, TRUE);  // キー入力ハンドルを作る
                                             // - Create key input handle
    maincolor = GetColor(50, 50, 50);
}

void InputBox::Update() {
    int x = Mouse::Instance()->GetX();
    int y = Mouse::Instance()->GetY();

    if (mX <= x && x <= mX + mW && mY <= y && y <= mY + mH) {
        IsMouseover = true;
        if (Mouse::Instance()->GetPressingCount(Mouse::LEFT) == 0) {
            IsPressed = false;
        }
        if (Mouse::Instance()->GetPressingCount(Mouse::LEFT) == 1) {
            IsPressed = true;
            // 作成したキー入力ハンドルをアクティブにする
            // - Activate the created key input handle
            SetActiveKeyInput(InputHandle);
            count = 0;
        }
    } else {
        IsMouseover = false;
        IsPressed = false;
    }
}

void InputBox::Draw() {
    int fontsize;
    GetFontStateToHandle(NULL, &fontsize, NULL, font);

    // 四角形を描画 - Draw rectangle
    DrawBoxAA(mX, mY, mX + mW, mY + mH, maincolor, FALSE,
              (IsMouseover || GetActiveKeyInput() == InputHandle) ? 5.5 : 2.5);

    // 入力途中の文字列を描画 - Draw string in middle of input
    // (?)
    char string[20];
    GetKeyInputString(string, InputHandle);
    int strX =
        mX + mW - GetDrawStringWidthToHandle(string, strlen(string), font) - 20;
    int strY = mY + mH / 2 - fontsize / 2;
    DrawFormatStringToHandle(strX, strY, maincolor, font, string);

    // カーソルを描画 - Draw cursor
    if (GetActiveKeyInput() == InputHandle) {
        int cursorX = strX
                    + GetDrawStringWidthToHandle(
                          string, GetKeyInputCursorPosition(InputHandle), font);
        if (count < 30) {
            DrawLineAA(cursorX, strY + 7, cursorX, strY + 50, maincolor, 3.5);
        }
        count++;
        if (count >= 60) {
            count = 0;
        }
    }
}

void InputBox::UpdateDraw() {
    Update();
    Draw();
}

float InputBox::GetNum() {
    return GetKeyInputNumberToFloat(InputHandle);
}

void InputBox::SetNum(float num) {
    char str[20];
    sprintf(str, "%.2f", num);
    SetKeyInputString(str, InputHandle);
}
