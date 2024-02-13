/******************************************************************************
 * @file   view.h
 * @brief  TODO.
 *
 * @author Yuto Goto, Christian Brice
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
//   (none)

/**
 * @brief TODO.
 */
class View {
  public:
    virtual ~View(){};

    virtual void Draw(){};
    virtual void Update(){};
};
