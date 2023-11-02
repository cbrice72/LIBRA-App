#pragma once

#include "View.h"
#include "OnClickListener.h"

class Button : public View {

public:
    Button(int x, int y, int w, int h, const char* str, OnClickListener* listener);
    void Update() override;
    void Draw() override;
    void UpdateDraw();

private:
    int mX, mY, mW, mH;
    const char *mStr;
    bool IsPressed;
    bool IsMouseover;
    OnClickListener* mListener;

    int font;
};
