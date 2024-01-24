/******************************************************************************
 * @file   camera.h
 * @brief  Camera controls header file. (UNUSED)
 *
 * @author Yuto Goto, Christian Brice
 * @date   ???
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   DirectX Wrapper
#include <DxLib.h>

// Project Headers
//   (none)

/**
 * @brief TODO.
 */
class Camera {
  public:
    Camera(int height, int width, int num);

    void Draw(int x1, int y1, int x2, int y2);

  private:
    // --- Data Members ---

    int camera_num_;
    unsigned char* buffer_;
    BASEIMAGE base_image_;
    int gfx_handle_;
};
