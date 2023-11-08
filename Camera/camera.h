/******************************************************************************
 * @file   camera.h
 * @brief  Camera controls header file.
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
//   (none)
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
    int camera_num;
    unsigned char* buffer;
    BASEIMAGE BaseImage;
    int GrHandle;
};
