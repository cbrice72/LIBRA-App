#include "AppMgr.h"
#include "Button.h"
#include "Mouse.h"
#include "Serial.h"
#include <DxLib.h>
#include <Math.h>
#include <ctime>
#include <direct.h>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <setupapi.h>
#pragma comment(lib, "setupapi.lib")

void AppMgr::Main() {
  // コンソールを用意 - Prepare console
  AllocConsole();
  (void)freopen("CONOUT$", "w", stdout);
  (void)freopen("CONIN$", "r", stdin);
  printf("\n==== LIBRA App Start ====\n\n");

  // HEBIアクチュエータ接続 - HEBI actuator connection
  ARM = new LIBRA_HEBI();
  int hebi_error = ARM->connect();

  // COMポート接続 - COM port connection
  SerialWater = new Serial();
  SerialServo = new Serial();
  if (!hebi_error) { // HEBIアクチュエータと接続されているときのみ - Only when
                     // connected to HEBI actuator
    int com;
    char comtext[10];
    std::string answer;
    do {
      printf("\n\n---- COMポート一覧 ----\n");
      int port = printComList();
      printf("\n%d個のポートが検出されました。\n", port);
      std::cout << "再検出しますか？[y/n]:";
      std::cin >> answer;
    } while (answer != "n");

    printf("\nSerialWaterのCOMポート番号を入力 : COM");
    (void)scanf("%d", &com);
    sprintf(comtext, "COM%d", com);
    if (SerialWater->open(comtext)) {
      printf("%sが開きません。\n", comtext);
    }

    printf("SerialServoのCOMポート番号を入力 : COM");
    (void)scanf("%d", &com);
    sprintf(comtext, "COM%d", com);
    if (SerialServo->open(comtext)) {
      printf("%sが開きません。\n", comtext);
    }
  }

  // DXライブラリ初期化を含む設定 - Setup, including DX library initialization
  printf("\nDxlibを起動します...");
  SetupIncludeDxlibInit();

  // ProcessMessage以外の処理を行うスレッドを作成 - Create threads for
  // processing (other than 'ProcessMessage')
  CreateThread(NULL, 0, MainThread_dmy, this, 0, NULL);

  // ProcessMessageループ - 'ProcessMessage' loop
  while (!ProcessMessage() && !ThreadEndFlag) {
    // 少しCPUを休める - Wait for the thread to finish
    Sleep(6);
  }

  // プログラムが終了したことを示すフラグを立てる - Flag to indicate that the
  // program has finished
  EndFlag = 1;

  // スレッド終了フラグが立つまで待つ - Wait until the thread is flagged as
  // closed
  while (!ThreadEndFlag) {
    Sleep(10);
  }

  DxLib_End();
}

void AppMgr::MainThread() {
  int maincolor = GetColor(50, 50, 50);
  int mainfont =
      CreateFontToHandle("Yu Gothic UI", 50, 5, DX_FONTTYPE_ANTIALIASING);
  int titlefont =
      CreateFontToHandle("Yu Gothic UI", 50, 10, DX_FONTTYPE_ANTIALIASING);
  int bigfont =
      CreateFontToHandle("Yu Gothic UI", 150, 10, DX_FONTTYPE_ANTIALIASING);
  SetBackgroundColor(255, 255, 255);

  InputBox_Roll = new InputBox(1500 - 600, 800);
  InputBox_Pitch = new InputBox(1500 - 600, 1000);
  InputBox_J1 = new InputBox(1500 - 600, 1200);
  InputBox_J2 = new InputBox(1500 - 600, 1400);
  InputBox_J3 = new InputBox(1500 - 600, 1600);
  InputBox_R = new InputBox(270, 1400);
  InputBox_Theta = new InputBox(270, 1600);
  InputBox_Increment = new InputBox(270, 1010);
  InputBox_Voltage = new InputBox(1400, 1900);
  InputBox_Current = new InputBox(1400 + 400, 1900);
  InputBox_CamPan = new InputBox(2800 + 150, 450);
  InputBox_CamTilt = new InputBox(2800 + 150, 600);

  InputBox_Roll->SetNum(ARM->getCommandPosition(LIBRA_HEBI::ROLL));
  InputBox_Pitch->SetNum(ARM->getCommandPosition(LIBRA_HEBI::PITCH));
  InputBox_J1->SetNum(ARM->getCommandPosition(LIBRA_HEBI::J1));
  InputBox_J2->SetNum(ARM->getCommandPosition(LIBRA_HEBI::J2));
  InputBox_J3->SetNum(ARM->getCommandPosition(LIBRA_HEBI::J3));
  InputBox_R->SetNum(1200);
  InputBox_Theta->SetNum(0);
  InputBox_Increment->SetNum(10);
  InputBox_Voltage->SetNum(24);
  InputBox_Current->SetNum(0);
  InputBox_CamPan->SetNum(0);
  InputBox_CamTilt->SetNum(0);

  StartButton = new Button(1300 - 60 - 340 - 60 - 330, WindowH - 200 - 100, 340,
                           100, "START", this);
  OnClick(StartButton);

  ConvertButton =
      new Button(60 + 60, WindowH - 200 - 100, 340, 100, "CONVERT", this);
  StopButton =
      new Button(1300 - 60 - 340, WindowH - 200 - 100, 340, 100, "STOP", this);
  UpButton = new Button(320, 800, 120, 120, "R+", this);
  DownButton = new Button(320, 1200, 120, 120, "R-", this);
  LeftButton = new Button(120, 1000, 120, 120, "θ+", this);
  RightButton = new Button(520, 1000, 120, 120, "θ-", this);
  EnableButton = new Button(2200, 150, 340, 100, "ENABLE", this);
  DisableButton = new Button(2200, 300, 340, 100, "DISABLE", this);
  ShotButton =
      new Button(2200, WindowH - 200 - 100, 340, 100, "LOG SHOT", this);
  ServoSlowButton = new Button(3230, 150, 200, 100, "SLOW", this);
  ServoFastButton = new Button(3500, 150, 200, 100, "FAST", this);

  int count = 0;
  BYTE d = 0;

  // ログ - Log

  SYSTEMTIME st;
  char datetime_char[100];
  std::string datetime_str;
  GetLocalTime(&st);
  sprintf(datetime_char, "%04d年%02d月%02d日_%02d時%02d分%02d秒", st.wYear,
          st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
  datetime_str = datetime_char;

  if (_mkdir("./Log/") == 0)
    printf("ログフォルダを作成しました。\n");

  continuous_log =
      new std::ofstream("./Log/" + datetime_str + "_continuous_log.csv");
  *continuous_log << "Time,,";
  *continuous_log
      << "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
  *continuous_log
      << "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
  *continuous_log << "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,";
  *continuous_log << "A_IN,B_IN,A_OUT,B_OUT,,";
  *continuous_log << "TP_CamBase[deg],TP_CamPan[deg],TP_CamTilt[deg]";
  *continuous_log << std::endl;

  shot_log = new std::ofstream("./Log/" + datetime_str + "_shot_log.csv");
  *shot_log << "Time,,";
  *shot_log << "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
  *shot_log << "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
  *shot_log << "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,";
  *shot_log << "Voltage[V],Current[A]";
  *shot_log << std::endl;

  //-----------------------------

  while (!EndFlag && !ScreenFlip() && !ClearDrawScreen()) {
    Mouse::Instance()->Update();

    InputBox_Roll->UpdateDraw();
    InputBox_Pitch->UpdateDraw();
    InputBox_J1->UpdateDraw();
    InputBox_J2->UpdateDraw();
    InputBox_J3->UpdateDraw();
    InputBox_R->UpdateDraw();
    InputBox_Theta->UpdateDraw();
    InputBox_Increment->UpdateDraw();
    InputBox_Voltage->UpdateDraw();
    InputBox_Current->UpdateDraw();
    InputBox_CamPan->UpdateDraw();
    InputBox_CamTilt->UpdateDraw();

    StartButton->UpdateDraw();
    StopButton->UpdateDraw();
    ConvertButton->UpdateDraw();
    UpButton->UpdateDraw();
    DownButton->UpdateDraw();
    LeftButton->UpdateDraw();
    RightButton->UpdateDraw();
    EnableButton->UpdateDraw();
    DisableButton->UpdateDraw();
    ShotButton->UpdateDraw();
    ServoSlowButton->UpdateDraw();
    ServoFastButton->UpdateDraw();

    // 操作盤の四角 - Operation panel square
    DrawBoxAA(60, 550, 1300, WindowH - 100, maincolor, FALSE, 2.5);

    // タイトル - Title
    DrawFormatStringToHandle(60, 200, GetColor(0, 0, 0), bigfont, "LIBRA-I");
    DrawFormatStringToHandle(120, 824 - 200, maincolor, titlefont, "Goal Pos.");
    DrawFormatStringToHandle(1400, 824 - 200, maincolor, titlefont,
                             "Target Pos.");
    DrawFormatStringToHandle(1400 + 400, 824 - 200, maincolor, titlefont,
                             "Present Pos.");
    DrawFormatStringToHandle(1400 + 800, 824 - 200, maincolor, titlefont,
                             "Present Torq.");

    DrawFormatStringToHandle(800, 200, maincolor, titlefont, "A_IN");
    DrawFormatStringToHandle(1100, 200, maincolor, titlefont, "B_IN");
    DrawFormatStringToHandle(1400, 200, maincolor, titlefont, "A_OUT");
    DrawFormatStringToHandle(1700, 200, maincolor, titlefont, "B_OUT");

    // 各項目 - Iterate over each item
    for (int i = 0; i < 5; i++) {
      value[i][0] = ARM->getCommandPosition(i);
      value[i][1] = ARM->getFeedbackPosition(i);
      value[i][2] = ARM->getFeedbackEffort(i);
    }

    std::string menu[] = {"Roll", "Pitch", "J1", "J2", "J3"};
    for (int i = 0; i < 5; i++) {
      DrawFormatStringToHandle(750, 824 + 200 * i, maincolor, mainfont,
                               menu[i].c_str());
      DrawFormatStringToHandle(1500 - 340, 824 + 200 * i, maincolor, mainfont,
                               "deg");

      for (int j = 0; j < 3; j++) {
        char str[20];
        int strW;
        sprintf(str, "%8.2f %s", value[i][j], j != 2 ? "deg" : "Nm");
        strW = GetDrawStringWidthToHandle(str, strlen(str), mainfont);
        DrawFormatStringToHandle(1750 + 400 * j - strW, 824 + 200 * i,
                                 maincolor, mainfont, str);
      }
    }

    DrawFormatStringToHandle(120, 824 + 200 * 3, maincolor, mainfont, "R");
    DrawFormatStringToHandle(530, 824 + 200 * 3, maincolor, mainfont, "mm");
    DrawFormatStringToHandle(120, 824 + 200 * 4, maincolor, mainfont, "θ");
    DrawFormatStringToHandle(530, 824 + 200 * 4, maincolor, mainfont, "deg");

    // グラフ表示 - Graph display
    const int cX = 3100, cY = WindowH / 2 + 350;
    DrawLineAA(cX + 40 * 0, cY - 40 * 10, cX + 40 * (-10), cY - 40 * 0,
               maincolor, 2.5);
    DrawLineAA(cX + 40 * -10, cY - 40 * 0, cX + 40 * (0), cY - 40 * -10,
               maincolor, 2.5);
    DrawLineAA(cX + 40 * 0, cY - 40 * -10, cX + 40 * (10), cY - 40 * 0,
               maincolor, 2.5);
    DrawLineAA(cX + 40 * 10, cY - 40 * 0, cX + 40 * (0), cY - 40 * 10,
               maincolor, 2.5);

    DrawLineAA(cX + 40 * (0), cY - 40 * (5), cX + 40 * (-5), cY - 40 * (0),
               maincolor, 2.5);
    DrawLineAA(cX + 40 * (-5), cY - 40 * (0), cX + 40 * (0), cY - 40 * (-5),
               maincolor, 2.5);
    DrawLineAA(cX + 40 * (0), cY - 40 * (-5), cX + 40 * (5), cY - 40 * (0),
               maincolor, 2.5);
    DrawLineAA(cX + 40 * (5), cY - 40 * (0), cX + 40 * (0), cY - 40 * (5),
               maincolor, 2.5);

    DrawLineAA(cX - 500, cY, cX + 500, cY, maincolor, 2.5);
    DrawLineAA(cX + 500 * cos(M_PI * 1 / 8), cY + 500 * sin(M_PI * 1 / 8),
               cX - 500 * cos(M_PI * 1 / 8), cY - 500 * sin(M_PI * 1 / 8),
               maincolor, 1);
    DrawLineAA(cX + 500 * cos(M_PI * 3 / 8), cY + 500 * sin(M_PI * 3 / 8),
               cX - 500 * cos(M_PI * 3 / 8), cY - 500 * sin(M_PI * 3 / 8),
               maincolor, 1);
    DrawLineAA(cX + 500 * cos(M_PI * 5 / 8), cY + 500 * sin(M_PI * 5 / 8),
               cX - 500 * cos(M_PI * 5 / 8), cY - 500 * sin(M_PI * 5 / 8),
               maincolor, 1);
    DrawLineAA(cX + 500 * cos(M_PI * 7 / 8), cY + 500 * sin(M_PI * 7 / 8),
               cX - 500 * cos(M_PI * 7 / 8), cY - 500 * sin(M_PI * 7 / 8),
               maincolor, 1);
    DrawLineAA(cX, cY - 500, cX, cY + 500, maincolor, 2.5);
    DrawTriangleAA(cX + 500, cY, cX + 480, cY + 10, cX + 480, cY - 10,
                   maincolor, TRUE);
    DrawTriangleAA(cX, cY - 500, cX + 10, cY - 480, cX - 10, cY - 480,
                   maincolor, TRUE);
    DrawCircleAA(cX + value[0][2] * 40, cY - value[1][2] * 40, 15, 20,
                 GetColor(0, 0, 0), TRUE);
    DrawFormatStringToHandle(cX + 525, cY - 25, maincolor, mainfont,
                             "Roll[Nm]");
    DrawFormatStringToHandle(cX - 100, cY - 550 - 25, maincolor, mainfont,
                             "Pitch[Nm]");

    // 電圧電流 - Voltage and current
    DrawFormatStringToHandle(1400, 1800, maincolor, titlefont, "Voltage");
    DrawFormatStringToHandle(1400 + 400, 1800, maincolor, titlefont, "Current");
    DrawFormatStringToHandle(1400 + 260, 1924, maincolor, mainfont, "V");
    DrawFormatStringToHandle(1400 + 400 + 260, 1924, maincolor, mainfont, "A");

    // カメラ - Camera
    DrawFormatStringToHandle(2800, 150, maincolor, titlefont, "Camera Pos.");
    DrawFormatStringToHandle(2800, 300 + 24, maincolor, mainfont, "Base");
    DrawFormatStringToHandle(2800, 450 + 24, maincolor, mainfont, "Pan");
    DrawFormatStringToHandle(2800, 600 + 24, maincolor, mainfont, "Tilt");
    DrawFormatStringToHandle(2800 + 410, 450 + 24, maincolor, mainfont, "deg");
    DrawFormatStringToHandle(2800 + 410, 600 + 24, maincolor, mainfont, "deg");

    // ???
    double j3_pos = ARM->getCommandPosition(4);
    if (j3_pos <= 30) {
      camera_pos[0] = (j3_pos <= 0) ? -j3_pos : 0;
    } else {
      camera_pos[0] = 180 - j3_pos;
    }

    // ???
    if (camera_dir[1]) {
      camera_pos[1] += camera_dir[1] * 90.0 / (60.0 * 60.0);
      if ((camera_pos[1] > camera_setpos[1]) == (camera_dir[1] == 1)) {
        camera_pos[1] = camera_setpos[1];
        camera_dir[1] = 0;
      }
    }

    // ???
    if (camera_dir[2]) {
      camera_pos[2] += camera_dir[2] * 90.0 / (60.0 * 60.0);
      if ((camera_pos[2] > camera_setpos[2]) == (camera_dir[2] == 1)) {
        camera_pos[2] = camera_setpos[2];
        camera_dir[2] = 0;
      }
    }

    // ???
    for (int i = 0; i < 3; i++) {
      char str[20] = "";
      int strW;
      sprintf(str, "%8.2f deg", camera_pos[i]);
      strW = GetDrawStringWidthToHandle(str, strlen(str), mainfont);
      DrawFormatStringToHandle(3670 - strW, 300 + 24 + 150 * i, maincolor,
                               mainfont, str);
    }

    std::stringstream servo_str;
    servo_str << camera_pos[0] << " " << camera_pos[1] << " " << camera_pos[2]
              << "\n";
    SerialServo->writestring(servo_str.str());

    // ???
    switch (mode) {
    case 0: // 通常運転 - Normal operation
      d = 0;
      // トルク超過 - Excess torque
      if ((abs(ARM->getFeedbackEffortMA()) > 5 ||
           abs(ARM->getFeedbackEffortMB()) > 5) &&
          enable) {
        ARM->stop();
        mode = 1;
      }
      break;

    case 1: // 調整 - Adjustment
      // 通常動作 - Usual action
      if ((abs(ARM->getFeedbackEffortMA()) >= 2.5 ||
           abs(ARM->getFeedbackEffortMB()) >= 2.5) &&
          enable) {
        if (count == 0) {
          double theta = atan2(ARM->getFeedbackEffort(LIBRA_HEBI::PITCH),
                               ARM->getFeedbackEffort(LIBRA_HEBI::ROLL));

          // A入 | B入 | A出 | B出 - A in | B in | A out | B out
          if (theta > M_PI * 7 / 8 || -M_PI * 7 / 8 >= theta)
            d = 0b1001;
          else if (theta > M_PI * 5 / 8)
            d = 0b0001;
          else if (theta > M_PI * 3 / 8)
            d = 0b0011;
          else if (theta > M_PI * 1 / 8)
            d = 0b0010;
          else if (theta > -M_PI * 1 / 8)
            d = 0b0110;
          else if (theta > -M_PI * 3 / 8)
            d = 0b0100;
          else if (theta > -M_PI * 5 / 8)
            d = 0b1100;
          else
            d = 0b1000;
        }
      }

      // トルクが戻った - Torque is reset
      else if (count == 0) {
        ARM->move(input[0], input[1], input[2], input[3], input[4]);
        mode = 0;
        d = 0;
      }
      break;
    }
    SerialWater->write(d);

    for (int i = 0; i < 4; i++) {
      if (!enable)
        DrawFormatStringToHandle(800 + 300 * i, 300, maincolor, mainfont,
                                 "DISABLE");
      else if (d & 1 << (3 - i))
        DrawFormatStringToHandle(800 + 300 * i, 300,
                                 i < 2 ? GetColor(50, 150, 50)
                                       : GetColor(0, 0, 255),
                                 titlefont, "ON");
      else
        DrawFormatStringToHandle(800 + 300 * i, 300, maincolor, mainfont,
                                 "OFF");
    }

    // ログ - Log

    if (count == 0) {
      /*
      continuous_log << "Time,,";
      continuous_log <<
      "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
      continuous_log <<
      "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
      continuous_log <<
      "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,"; continuous_log
      << "A_IN,B_IN,A_OUT,B_OUT,,"; continuous_log <<
      "TP_CamBase[deg],TP_CamPan[deg],TP_CamTilt[deg]"; continuous_log <<
      std::endl;
      */

      *continuous_log << GetDateTimeString() + ",,";
      for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 5; i++) {
          *continuous_log << value[i][j];
          *continuous_log << ",";
        }
        *continuous_log << ",";
      }
      for (int b = 0; b < 4; b++) {
        *continuous_log << ((d & (1 << (3 - b))) ? 1 : 0);
        *continuous_log << ",";
      }
      *continuous_log << ",";
      *continuous_log << camera_pos[0] << "," << camera_pos[1] << ","
                      << camera_pos[2];
      *continuous_log << std::endl;
    }

    //--------------------

    count++;
    if (count == 30)
      count = 0;
  }
}

void AppMgr::OnClick(View *view) {
  if (view == StopButton) {
    ARM->stop();
  }

  if (view == StartButton) {
    input[0] = InputBox_Roll->GetNum();
    input[1] = InputBox_Pitch->GetNum();
    input[2] = InputBox_J1->GetNum();
    input[3] = InputBox_J2->GetNum();
    input[4] = InputBox_J3->GetNum();
    if (mode == 0)
      ARM->move(input[0], input[1], input[2], input[3], input[4]);
  }

  if (view == ConvertButton) {
    double r = InputBox_R->GetNum();
    double theta = InputBox_Theta->GetNum();
    double L = 989;
    double L_hand = 1014;
    double a = L;
    double b = L + L_hand;
    double k = (r * r + a * a - b * b) / (2 * a);
    double alpha = (atan2(0, r) + atan2(sqrt(r * r - k * k), k));
    double beta = asin(r * sin(alpha) / b);
    InputBox_J1->SetNum(theta + alpha / M_PI * 180);
    InputBox_J2->SetNum(-180 + beta / M_PI * 180);
    InputBox_J3->SetNum(0);
  }

  if (view == UpButton) {
    InputBox_R->SetNum(InputBox_R->GetNum() + InputBox_Increment->GetNum());
    OnClick(ConvertButton);
  }

  if (view == DownButton) {
    InputBox_R->SetNum(InputBox_R->GetNum() - InputBox_Increment->GetNum());
    OnClick(ConvertButton);
  }

  if (view == LeftButton) {
    InputBox_Theta->SetNum(InputBox_Theta->GetNum() +
                           InputBox_Increment->GetNum() / InputBox_R->GetNum() *
                               180 / M_PI);
    OnClick(ConvertButton);
  }

  if (view == RightButton) {
    InputBox_Theta->SetNum(InputBox_Theta->GetNum() -
                           InputBox_Increment->GetNum() / InputBox_R->GetNum() *
                               180 / M_PI);
    OnClick(ConvertButton);
  }

  if (view == EnableButton)
    enable = true;
  if (view == DisableButton)
    enable = false;
  if (view == ShotButton) {
    /*
    shot_log << "Time,,";
    shot_log << "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
    shot_log << "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
    shot_log << "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,";
    shot_log << "Voltage[V],Current[A]";
    */
    std::string gtds = GetDateTimeString();
    *shot_log << gtds + ",,";
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 5; i++) {
        *shot_log << value[i][j];
        *shot_log << ",";
      }
      *shot_log << ",";
    }
    *shot_log << InputBox_Voltage->GetNum();
    *shot_log << ",";
    *shot_log << InputBox_Current->GetNum();
    *shot_log << std::endl;
    printf("** LOG SHOT **  %s  Voltage: %lf V  Current: %lf A\n", gtds.c_str(),
           InputBox_Voltage->GetNum(), InputBox_Current->GetNum());
  }

  if (view == ServoFastButton) {
    camera_pos[1] = InputBox_CamPan->GetNum();
    camera_pos[2] = InputBox_CamTilt->GetNum();
  }

  if (view == ServoSlowButton) {
    camera_setpos[1] = InputBox_CamPan->GetNum();
    camera_setpos[2] = InputBox_CamTilt->GetNum();
    camera_dir[1] = (InputBox_CamPan->GetNum() >= camera_pos[1]) ? 1 : -1;
    camera_dir[2] = (InputBox_CamTilt->GetNum() >= camera_pos[2]) ? 1 : -1;
  }
}

DWORD WINAPI AppMgr::MainThread_dmy(LPVOID pv) {
  AppMgr *p = (AppMgr *)pv;
  p->MainThread();
  p->ThreadEndFlag =
      1; // スレッド終了フラグを１にする - Set thread end flag to 1
  return 0;
}

void AppMgr::SetupIncludeDxlibInit() {

  // ウインドウモードで起動 - Start in windowed mode
  ChangeWindowMode(TRUE);

  // 最大化ボタンが存在するウインドウモードに変更 - Set to windowed mode with
  // maximize button present
  SetWindowStyleMode(7);

  // 画面サイズを指定 - Specify screen size
  SetGraphMode(WindowW, WindowH, 32);

  // サイズ変更を可能にする - Allow resizing
  SetWindowSizeChangeEnableFlag(TRUE, TRUE);

  // ウインドウサイズを指定 - Specify window size
  int DesktopW, DesktopH;
  GetDefaultState(&DesktopW, &DesktopH, NULL);

  // 横長ディスプレイ - Landscape display
  if ((float)DesktopW / DesktopH > (float)WindowW / WindowH) {
    SetWindowSize(0.8 * DesktopH * (WindowW / WindowH), 0.8 * DesktopH);
  }
  // 縦長ディスプレイ - Portrait display
  else {
    SetWindowSize(0.8 * DesktopW, 0.8 * DesktopW * (WindowH / WindowW));
  }

  // ウィンドウがノンアクティブでも実行 - Execute even if window is inactive
  SetAlwaysRunFlag(TRUE);

  // マルチスレッドに適したモードで起動する - Start in mode suitable for
  // multi-threading
  SetMultiThreadFlag(TRUE);

  // DXライブラリでWM_PAINTの処理をしない - Do not process WM_PAINT in the DX
  // library
  SetUseDxLibWM_PAINTProcess(FALSE);

  // Windowのタイトルを設定 - Set the window title
  SetWindowText("LIBRA App");

  // DXライブラリの初期化 - Initialize DX library
  DxLib_Init();

  // 描画先を裏画面にする - Draw the back screen (?)
  SetDrawScreen(DX_SCREEN_BACK);

  // アンチエイリアス付き図形描画の準備を行う - Prepare for drawing anti-aliased
  // shapes
  BeginAADraw();
}

std::string AppMgr::GetDateTimeString() {
  SYSTEMTIME st;
  char datetime_char[100];
  GetLocalTime(&st);
  sprintf(datetime_char, "%04d/%02d/%02d %02d:%02d:%02d.%03d", st.wYear,
          st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
          st.wMilliseconds);
  return (std::string)datetime_char;
}

int AppMgr::printComList(void) {
  HDEVINFO hDevInfo;
  DWORD MemberIndex = 0;
  SP_DEVINFO_DATA Data = {sizeof(SP_DEVINFO_DATA)};

  int max = 0;
  // デバイス情報セットを取得 - Get device information set
  hDevInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, 0, 0,
                                 DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (hDevInfo == 0) // デバイス情報セットが取得できなかった場合 - If the device
                     // information set could not be obtained
    return 0;
  Data.cbSize = sizeof(Data);

  while (SetupDiEnumDeviceInfo(
      hDevInfo, max,
      &Data)) { // デバイスインターフェイスの取得 - Get device interface
    DWORD dataT;
    DWORD size;
    LPTSTR buf;

    // COMポート名の取得 - Obtain COM port name
    HKEY key = SetupDiOpenDevRegKey(hDevInfo, &Data, DICS_FLAG_GLOBAL, 0,
                                    DIREG_DEV, KEY_QUERY_VALUE);
    if (key) {
      TCHAR name[256];
      DWORD type = 0;
      size = sizeof(name);
      RegQueryValueEx(key, _T("PortName"), NULL, &type, (LPBYTE)name, &size);
      _tprintf(_TEXT("%s"), name);
    }

    // デバイスの説明を取得 - Get device description
    size = 0;
    buf = NULL;
    while (!SetupDiGetDeviceRegistryProperty(hDevInfo, &Data, SPDRP_DEVICEDESC,
                                             &dataT, (PBYTE)buf, size, &size)) {
      if (GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
        if (buf)
          LocalFree(buf);
        buf = (LPTSTR)LocalAlloc(LPTR, size * 2);
      } else
        break;
    }

    _tprintf(_TEXT("(%s)\n"), buf);
    if (buf)
      LocalFree(buf);
    ++max;
  }

  SetupDiDestroyDeviceInfoList(
      hDevInfo); // デバイス情報セットを解放 - Release device information set
  return max;
}
