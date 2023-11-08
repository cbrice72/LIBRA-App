/******************************************************************************
 * @file   OnClickListener.h
 * @brief  TODO.
 *
 * @author Yuto Goto
 * @date   2022/1/13
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   (none)
// Project Headers
#include "View.h"

/**
 * @brief TODO.
 */
class OnClickListener {
  public:
    virtual ~OnClickListener() {}

    virtual void OnClick(View* view) {}
};
