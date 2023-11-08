/******************************************************************************
 * @file   camera.cpp
 * @brief  Camera controls implementation file.
 *
 * @author Yuto Goto, Christian Brice
 * @date   ???
 ******************************************************************************/

// Related Header
#include "camera.h"
// C++ Standard Library Headers
#include <iostream>
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   COM Library
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "OleAut32.lib ")
//   Easy Web Camera LIBrary
#include "ewclib.h"

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !General Functions
 */

//------------------------------------------------------------------------------
// !General Functions
//------------------------------------------------------------------------------

/**
 * @brief Constructs a new Camera object.
 *
 * @param height TODO
 * @param width TODO
 * @param num TODO
 */
Camera::Camera(int height, int width, int num) {
    camera_num_ = num;

    // EWCLIB初期化 - Initialize EWCLIB
    ewc_camera ewcc[10];
    int n = 10;

    EWC_GetCameraName(ewcc, &n);
    for (int i = 0; i < n; i++) {
        std::cout << ewcc[i].FriendlyName << "\n";
    }
    std::cout << "[INFO] Camera - Available: " << EWC_GetCamera() << "\n";
    std::cout << "[INFO] Camera - Open Error: "
              << EWC_Open(camera_num_, height, width, 30.0, -1,
                          MEDIASUBTYPE_RGB24)
              << std::endl;

    // 画像変換用 - Initialize image conversion
    buffer_ = new unsigned char[height * width * 3];
    //   BASEIMAGEの要素を埋める - Populate the BASEIMAGE structure
    memset(&base_image_, 0, sizeof(BASEIMAGE));
    base_image_.GraphData = buffer_;
    base_image_.Width = height;
    base_image_.Height = width;
    base_image_.Pitch = base_image_.Width * 3;
    base_image_.MipMapCount = 0;
    CreateFullColorData(&base_image_.ColorData);

    // 空のグラフィックハンドルの値を初期化 - Initialize empty graphics handle
    gfx_handle_ = -1;
}

/**
 * @brief TODO.
 *
 * @param x1 TODO
 * @param y1 TODO
 * @param x2 TODO
 * @param y2 TODO
 */
void Camera::Draw(int x1, int y1, int x2, int y2) {
    // 画像取得 - Image acquisition
    EWC_GetImage(camera_num_, buffer_);

    // グラフィックハンドルを作成しているかどうかで処理を分岐
    // Split processing based on whether or not a graphics handle is created
    if (gfx_handle_ == -1) {
        // 最初の場合はグラフィックハンドルの作成と映像の転送を一度に行う
        // The first time, create the graphics handle and transmit the video
        gfx_handle_ = CreateGraphFromBaseImage(&base_image_);
    } else {
        // ２回目以降はグラフィックハンドルへ映像を転送
        // From the second time onwards, transmit images to the graphics handle
        ReCreateGraphFromBaseImage(&base_image_, gfx_handle_);
    }

    // 変換した画像画面に描画 - Draw on the converted image display
    DrawExtendGraph(x1, y1, x2, y2, gfx_handle_, FALSE);
}
