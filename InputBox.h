#pragma once

#include "View.h"
#include "OnClickListener.h"

class InputBox : public View {

public:
    InputBox(int x, int y);
    void Update() override;
    void Draw() override;
    void UpdateDraw();
    float GetNum();
    void SetNum(float num);

private:
    int mX, mY, mW, mH;
    bool IsPressed;
    bool IsMouseover;

    int font;
    int InputHandle;
    int maincolor;
    int count=0;
};
