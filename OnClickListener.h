#pragma once

#include "View.h"

class OnClickListener {
public:
    virtual ~OnClickListener(){}
    virtual void OnClick(View* view){}
};
