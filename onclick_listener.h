/******************************************************************************
 * @file   onclick_listener.h
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
#include "view.h"

/**
 * @brief TODO.
 */
class OnClickListener {
  public:
    virtual ~OnClickListener(){};

    virtual void OnClick(View* view){};
};
