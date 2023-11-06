#pragma once

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
