/******************************************************************************
 * @file   Camera.cpp
 * @brief  Camera controls implementation file.
 *
 * @author Yuto Goto
 * @date   ???
 ******************************************************************************/

// Related Header
#include "Camera.h"
// C++ Standard Library Headers
//   (none)
// POSIX/Windows Library Headers
//   (none)
// Other Libraries' Headers
//   COM Library
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "OleAut32.lib ")
//   DirectX Wrapper
#include <DxLib.h>
//   Easy Web Camera LIBrary
#include "ewclib.h"

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !...
 */

/**
 * @brief Constructs a new Camera object.
 *
 * @param height TODO
 * @param width TODO
 * @param num TODO
 */
Camera::Camera(int height, int width, int num) {
    camera_num = num;

    // EWCLIB初期化 - Initialize EWCLIB
    ewc_camera ewcc[10];
    int n = 10;
    EWC_GetCameraName(ewcc, &n);
    for (int i = 0; i < n; i++) {
        printf("%s\n", ewcc[i].FriendlyName);
    }
    printf("台数:%d\n", EWC_GetCamera());
    printf("Openエラー:%d\n",
           EWC_Open(camera_num, height, width, 30.0, -1, MEDIASUBTYPE_RGB24));

    // 画像変換用 - Initialize image conversion
    buffer = new unsigned char[height * width * 3];
    //   BASEIMAGEの要素を埋める - Populate the BASEIMAGE structure
    memset(&BaseImage, 0, sizeof(BASEIMAGE));
    BaseImage.GraphData = buffer;
    BaseImage.Width = height;
    BaseImage.Height = width;
    BaseImage.Pitch = BaseImage.Width * 3;
    BaseImage.MipMapCount = 0;
    CreateFullColorData(&BaseImage.ColorData);

    // 空のグラフィックハンドルの値を初期化 - Initialize empty graphics handle
    GrHandle = -1;
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
    EWC_GetImage(camera_num, buffer);

    // グラフィックハンドルを作成しているかどうかで処理を分岐
    // Split processing based on whether or not a graphics handle is created
    if (GrHandle == -1) {
        // 最初の場合はグラフィックハンドルの作成と映像の転送を一度に行う
        // The first time, create the graphics handle and transmit the video
        GrHandle = CreateGraphFromBaseImage(&BaseImage);
    } else {
        // ２回目以降はグラフィックハンドルへ映像を転送
        // From the second time onwards, transmit images to the graphics handle
        ReCreateGraphFromBaseImage(&BaseImage, GrHandle);
    }

    // 変換した画像画面に描画 - Draw on the converted image display
    DrawExtendGraph(x1, y1, x2, y2, GrHandle, FALSE);
}
