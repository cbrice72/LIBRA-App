#include <DxLib.h>
#include "Camera.h"
#pragma comment(lib,"ole32.lib")
#pragma comment(lib,"OleAut32.lib ")
#include "ewclib.h"

Camera::Camera(int height,int width,int num){
	camera_num = num;
	
	//EWCLIB初期化
	ewc_camera ewcc[10];
	int n = 10;
	EWC_GetCameraName(ewcc, &n);
	for (int i = 0; i < n; i++) printf("%s\n", ewcc[i].FriendlyName);
	printf("台数:%d\n", EWC_GetCamera());
	printf("Openエラー:%d\n", EWC_Open(camera_num, height, width, 30.0, -1, MEDIASUBTYPE_RGB24));

	//画像変換用
	buffer= new unsigned char [height * width * 3];

	//画像変換用初期化
	// BASEIMAGE の要素を埋める
	memset(&BaseImage, 0, sizeof(BASEIMAGE));
	BaseImage.GraphData = buffer;
	BaseImage.Width = height;
	BaseImage.Height = width;
	BaseImage.Pitch = BaseImage.Width * 3;
	BaseImage.MipMapCount = 0;
	CreateFullColorData(&BaseImage.ColorData);

	// 空のグラフィックハンドルの値を初期化
	GrHandle = -1;
}

void Camera::Draw(int x1,int y1,int x2,int y2){
	//画像取得
	EWC_GetImage(camera_num, buffer);

	// グラフィックハンドルを作成しているかどうかで処理を分岐
	if (GrHandle == -1)
	{
		// 最初の場合はグラフィックハンドルの作成と映像の転送を一度に行う
		GrHandle = CreateGraphFromBaseImage(&BaseImage);
	}
	else
	{
		// ２回目以降はグラフィックハンドルへ映像を転送
		ReCreateGraphFromBaseImage(&BaseImage, GrHandle);
	}

	//変換した画像画面に描画
	DrawExtendGraph(x1, y1, x2, y2, GrHandle, FALSE);
}