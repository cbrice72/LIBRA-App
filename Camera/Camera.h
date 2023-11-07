/******************************************************************************
 * @file   .h
 * @brief  TODO
 *
 * @author Yuto Goto
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

class Camera {
  private:
    unsigned char* buffer;
    int GrHandle;
    BASEIMAGE BaseImage;
    int camera_num;

  public:
    Camera(int height, int width, int num);
    void Draw(int x1, int y1, int x2, int y2);
};
