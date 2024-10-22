/******************************************************************************
 * @file   singleton.h
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
//   (none)

/**
 * @brief TODO.
 *
 * @tparam _T TODO
 */
template<typename _T>
class Singleton {
  public:
    static _T* Instance() {
        static _T inst;
        return &inst;
    };

  protected:
    Singleton(){};
    virtual ~Singleton(){};

    // Copy constructor + assignment operator
    Singleton(const Singleton& r){};
    Singleton& operator=(const Singleton& r){};
};
