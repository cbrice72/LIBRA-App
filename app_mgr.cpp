/******************************************************************************
 * @file   app_mgr.cpp
 * @brief  Application manager implementation file.
 *
 * @author Yuto Goto, Christian Brice
 * @date   ???
 ******************************************************************************/

// Related Header
#include "app_mgr.h"
// C++ Standard Library Headers
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
// POSIX/Windows Library Headers
#include <direct.h>
#include <setupapi.h>
#pragma comment(lib, "setupapi.lib")
// Other Libraries' Headers
//   DirectX Wrapper
#include <DxLib.h>
// Project Headers
#include "button.h"
#include "mouse.h"
#include "serial.h"

/* --- TABLE OF CONTENTS ---
 * !Main Window
 * !Helper Functions
 */

constexpr int kWindowW = 1920 * 2;  // app window width
constexpr int kWindowH = 1080 * 2;  // app window height

//------------------------------------------------------------------------------
// !Main Window
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 */
void AppMgr::Main() {
    // コンソールを用意 - Prepare console
    AllocConsole();
    (void)freopen("CONOUT$", "w", stdout);
    (void)freopen("CONIN$", "r", stdin);
    std::cout << "\n===== LIBRA App =====\n" << std::endl;

    // HEBIアクチュエータ接続 - HEBI actuator connection
    libra_arm_ = new LIBRA_HEBI();
    int hebi_error = libra_arm_->connect();

    // COMポート接続 - COM port connection
    ser_water_ = new Serial();
    ser_servo_ = new Serial();
    if (!hebi_error) {  // HEBIアクチュエータと接続されているときのみ
                        // Only when connected to HEBI actuators
        std::string answer;
        do {
            std::cout << "----- COM Port List -----\n";
            int port = printComList();
            std::cout << "[INFO] Detected " << port << " ports\n";
            std::cout << "Would you like to scan again? [y/n]:" << std::flush;
            std::cin >> answer;
        } while (answer != "n");

        std::string com;
        std::string comtext;

        std::cout << "Specify the port to be used by SerialWater: "
                  << std::flush;
        std::cin >> com;
        comtext = "COM" + com;
        if (ser_water_->Open(comtext.c_str()) != 0) {
            std::cout << "[ERROR] Cannot open " << comtext << std::endl;
        }

        std::cout << "Specify the port to be used by SerialServo: "
                  << std::flush;
        std::cin >> com;
        comtext = "COM" + com;
        if (ser_servo_->Open(comtext.c_str()) != 0) {
            std::cout << "[ERROR] Cannot open " << comtext << std::endl;
        }
    }

    // DXライブラリ初期化を含む設定
    // DX library initialization
    std::cout << "[INFO] Initializing Dxlib...\n";
    SetupIncludeDxlibInit();

    // ProcessMessage以外の処理を行うスレッドを作成
    // Create thread for any non-ProcessMessage processing
    CreateThread(NULL, 0, MainThread_dmy, this, 0, NULL);

    // ProcessMessageループ
    // ProcessMessage loop
    while (!ProcessMessage() && !flag_thread_end_) {
        // 少しCPUを休める - Wait for the thread to finish
        Sleep(6);
    }

    // プログラムが終了したことを示すフラグを立てる
    // Flag to indicate that the program has finished
    flag_end_ = 1;

    // スレッド終了フラグが立つまで待つ
    // Wait until the thread is flagged as closed
    while (!flag_thread_end_) {
        Sleep(10);
    }

    // Clean up DX library
    DxLib_End();
}

/**
 * @brief TODO.
 */
DWORD WINAPI AppMgr::MainThread_dmy(LPVOID pv) {
    AppMgr* p = (AppMgr*)pv;
    p->MainThread();
    p->flag_thread_end_ = 1;
    return 0;
}

/**
 * @brief The primary program control loop; initializes the UI and continuously
 * receives user inputs & updates sensor readings.
 */
void AppMgr::MainThread() {
    // カラーとフォントの初期化
    // Initialize colors and fonts
    int maincolor = GetColor(50, 50, 50);
    int mainfont = CreateFontToHandle("Yu Gothic UI", 50, 5,
                                      DX_FONTTYPE_ANTIALIASING);
    int titlefont = CreateFontToHandle("Yu Gothic UI", 50, 10,
                                       DX_FONTTYPE_ANTIALIASING);
    int bigfont = CreateFontToHandle("Yu Gothic UI", 150, 10,
                                     DX_FONTTYPE_ANTIALIASING);
    SetBackgroundColor(255, 255, 255);

    // InputBoxオブジェクトの初期化
    // Initialize InputBox objects
    ibox_roll_ = new InputBox(1500 - 600, 800);
    ibox_pitch_ = new InputBox(1500 - 600, 1000);
    ibox_j1_ = new InputBox(1500 - 600, 1200);
    ibox_j2_ = new InputBox(1500 - 600, 1400);
    ibox_j3_ = new InputBox(1500 - 600, 1600);
    ibox_r_ = new InputBox(270, 1400);
    ibox_theta_ = new InputBox(270, 1600);
    ibox_increment_ = new InputBox(270, 1010);
    ibox_voltage_ = new InputBox(1400, 1900);
    ibox_current_ = new InputBox(1400 + 400, 1900);
    ibox_camera_pan_ = new InputBox(2800 + 150, 450);
    ibox_camera_tilt_ = new InputBox(2800 + 150, 600);

    // InputBoxのデフォルト値を設定
    // Set InputBox default values
    ibox_roll_->SetNum(libra_arm_->getCommandPosition(LIBRA_HEBI::Joint::kRoll));
    ibox_pitch_->SetNum(
        libra_arm_->getCommandPosition(LIBRA_HEBI::Joint::kPitch));
    ibox_j1_->SetNum(libra_arm_->getCommandPosition(LIBRA_HEBI::Joint::kJ1));
    ibox_j2_->SetNum(libra_arm_->getCommandPosition(LIBRA_HEBI::Joint::kJ2));
    ibox_j3_->SetNum(libra_arm_->getCommandPosition(LIBRA_HEBI::Joint::kJ3));
    ibox_r_->SetNum(1200);
    ibox_theta_->SetNum(0);
    ibox_increment_->SetNum(10);
    ibox_voltage_->SetNum(24);
    ibox_current_->SetNum(0);
    ibox_camera_pan_->SetNum(0);
    ibox_camera_tilt_->SetNum(0);

    // Buttonオブジェクトの初期化
    // Initialize Button objects
    btn_start_ = new Button(1300 - 60 - 340 - 60 - 330, kWindowH - 200 - 100,
                            340, 100, "START", this);
    OnClick(btn_start_);  // set Start button as default selection

    btn_convert_ = new Button(60 + 60, kWindowH - 200 - 100, 340, 100,
                              "CONVERT", this);
    btn_stop_ = new Button(1300 - 60 - 340, kWindowH - 200 - 100, 340, 100,
                           "STOP", this);
    btn_up_ = new Button(320, 800, 120, 120, "R+", this);
    btn_down_ = new Button(320, 1200, 120, 120, "R-", this);
    btn_left_ = new Button(120, 1000, 120, 120, "θ+", this);
    btn_right_ = new Button(520, 1000, 120, 120, "θ-", this);
    btn_enable_ = new Button(2200, 100, 340, 100, "ENABLE", this);
    btn_disable_ = new Button(2200, 250, 340, 100, "DISABLE", this);
    btn_drain_ = new Button(2200, 400, 340, 100, "DRAIN", this);
    btn_shot_ = new Button(2200, kWindowH - 200 - 100, 340, 100, "LOG SHOT",
                           this);
    btn_servo_slow_ = new Button(3230, 150, 200, 100, "SLOW", this);
    btn_servo_fast_ = new Button(3500, 150, 200, 100, "FAST", this);

    // 流体システムランタイム変数の初期化
    // Initialize fluid system runtime variables
    int count = 0;
    BYTE water_cmd = 0;

    // ロギングの初期化
    // Initialize logging
    SYSTEMTIME st;
    char dt_path_char[100];
    std::string dt_path_str;
    GetLocalTime(&st);
    sprintf(dt_path_char, "%04d年%02d月%02d日_%02d時%02d分%02d秒", st.wYear,
            st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    dt_path_str = std::string(dt_path_char);

    if (_mkdir("./log/") == 0) {
        std::cout << "[INFO] Created log directory.\n";
    }

    continuous_log_ = new std::ofstream("./log/" + dt_path_str
                                        + "_continuous_log.csv");
    *continuous_log_ << "Time,,";
    *continuous_log_
        << "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
    *continuous_log_
        << "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
    *continuous_log_
        << "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,";
    *continuous_log_ << "A_IN,B_IN,A_OUT,B_OUT,,";
    *continuous_log_ << "TP_CamBase[deg],TP_CamPan[deg],TP_CamTilt[deg]";
    *continuous_log_ << std::endl;

    shot_log_ = new std::ofstream("./log/" + dt_path_str + "_shot_log.csv");
    *shot_log_ << "Time,,";
    *shot_log_
        << "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
    *shot_log_
        << "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
    *shot_log_ << "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,";
    *shot_log_ << "Voltage[V],Current[A]";
    *shot_log_ << std::endl;

    // 更新ループ
    // Update loop
    while (!flag_end_ && ScreenFlip() == 0 && ClearDrawScreen() == 0) {
        Mouse::Instance()->Update();

        /* ----- UI: InputBox OBJECTS ----- */

        ibox_voltage_->UpdateDraw();
        ibox_current_->UpdateDraw();
        ibox_roll_->UpdateDraw();
        ibox_pitch_->UpdateDraw();
        ibox_j1_->UpdateDraw();
        ibox_j2_->UpdateDraw();
        ibox_j3_->UpdateDraw();
        ibox_increment_->UpdateDraw();
        ibox_r_->UpdateDraw();
        ibox_theta_->UpdateDraw();
        ibox_camera_pan_->UpdateDraw();
        ibox_camera_tilt_->UpdateDraw();

        /* ----- UI: Button OBJECTS ----- */

        btn_enable_->UpdateDraw();
        btn_disable_->UpdateDraw();
        btn_drain_->UpdateDraw();
        btn_shot_->UpdateDraw();
        btn_convert_->UpdateDraw();
        btn_start_->UpdateDraw();
        btn_stop_->UpdateDraw();
        btn_up_->UpdateDraw();
        btn_down_->UpdateDraw();
        btn_left_->UpdateDraw();
        btn_right_->UpdateDraw();
        btn_servo_slow_->UpdateDraw();
        btn_servo_fast_->UpdateDraw();

        /* ----- UI: GENERAL ----- */

        // NOLINTBEGIN(readability-magic-numbers): Positions of UI elements

        // 各セクションのタイトル
        // Section titles
        DrawFormatStringToHandle(60, 200, GetColor(0, 0, 0), bigfont, "LIBRA-I");
        DrawFormatStringToHandle(120, 824 - 200, maincolor, titlefont,
                                 "Goal Pos.");
        DrawFormatStringToHandle(1400, 824 - 200, maincolor, titlefont,
                                 "Target Pos.");
        DrawFormatStringToHandle(1400 + 400, 824 - 200, maincolor, titlefont,
                                 "Current Pos.");
        DrawFormatStringToHandle(1400 + 800, 824 - 200, maincolor, titlefont,
                                 "Current Torque");

        /* ----- UI: FLUID SYSTEM ----- */

        // 流体入出力パネルのラベル
        // Fluid I/O panel labels
        DrawFormatStringToHandle(800, 200, maincolor, titlefont, "A_IN");
        DrawFormatStringToHandle(1100, 200, maincolor, titlefont, "B_IN");
        DrawFormatStringToHandle(1400, 200, maincolor, titlefont, "A_OUT");
        DrawFormatStringToHandle(1700, 200, maincolor, titlefont, "B_OUT");

        /* ----- UI: ARM ----- */

        // 操作盤の四角
        // Control panel border
        DrawBoxAA(60, 550, 1300, kWindowH - 100, maincolor, FALSE, 2.5);

        // HEBIアクチュエータのデータを取得
        // Retrieve HEBI actuator data
        for (int i = 0; i < 5; i++) {
            auto joint = static_cast<LIBRA_HEBI::Joint>(i);
            value_[i][0] = libra_arm_->getCommandPosition(joint);
            value_[i][1] = libra_arm_->getFeedbackPosition(joint);
            value_[i][2] = libra_arm_->getFeedbackEffort(joint);
        }

        // アーム制御のラベルとアクチュエータデータ
        // Arm control labels and actuator data
        std::string menu[] = {"Roll", "Pitch", "J1", "J2", "J3"};
        for (int i = 0; i < 5; i++) {
            // Input box labels
            DrawFormatStringToHandle(750, 824 + 200 * i, maincolor, mainfont,
                                     menu[i].c_str());
            DrawFormatStringToHandle(1500 - 340, 824 + 200 * i, maincolor,
                                     mainfont, "deg");

            // Target Pos., Current Pos., Current Torque
            for (int j = 0; j < 3; j++) {
                char str[20];
                int strW;
                sprintf(str, "%8.2f %s", value_[i][j], j != 2 ? "deg" : "Nm");
                strW = GetDrawStringWidthToHandle(str, strlen(str), mainfont);
                DrawFormatStringToHandle(1750 + 400 * j - strW, 824 + 200 * i,
                                         maincolor, mainfont, str);
            }
        }

        // アーム全体コントロールのラベル
        // Whole-arm control labels
        DrawFormatStringToHandle(120, 824 + 200 * 3, maincolor, mainfont, "R");
        DrawFormatStringToHandle(530, 824 + 200 * 3, maincolor, mainfont, "mm");
        DrawFormatStringToHandle(120, 824 + 200 * 4, maincolor, mainfont, "θ");
        DrawFormatStringToHandle(530, 824 + 200 * 4, maincolor, mainfont, "deg");

        // 重心グラフ表示
        // Center of mass visualization
        const int cX = 3100;
        const int cY = kWindowH / 2 + 350;

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
        DrawCircleAA(cX + value_[0][2] * 40, cY - value_[1][2] * 40, 15, 20,
                     GetColor(0, 0, 0), TRUE);
        DrawFormatStringToHandle(cX + 525, cY - 25, maincolor, mainfont,
                                 "Roll[Nm]");
        DrawFormatStringToHandle(cX - 100, cY - 550 - 25, maincolor, mainfont,
                                 "Pitch[Nm]");

        // 電圧電流のラベル
        // Voltage and current labels
        DrawFormatStringToHandle(1400, 1800, maincolor, titlefont, "Voltage");
        DrawFormatStringToHandle(1400 + 400, 1800, maincolor, titlefont,
                                 "Current");
        DrawFormatStringToHandle(1400 + 260, 1924, maincolor, mainfont, "V");
        DrawFormatStringToHandle(1400 + 400 + 260, 1924, maincolor, mainfont,
                                 "A");

        /* ----- UI: CAMERA ----- */

        // カメラパネルのラベル
        // Camera panel labels
        DrawFormatStringToHandle(2800, 150, maincolor, titlefont, "Camera Pos.");
        DrawFormatStringToHandle(2800, 300 + 24, maincolor, mainfont, "Base");
        DrawFormatStringToHandle(2800, 450 + 24, maincolor, mainfont, "Pan");
        DrawFormatStringToHandle(2800, 600 + 24, maincolor, mainfont, "Tilt");
        DrawFormatStringToHandle(2800 + 410, 450 + 24, maincolor, mainfont,
                                 "deg");
        DrawFormatStringToHandle(2800 + 410, 600 + 24, maincolor, mainfont,
                                 "deg");

        // NOLINTEND(readability-magic-numbers): Positions of UI elements

        // J3を負にするピッチ角を計算
        // Calculate pitch angle to negate J3
        double j3_pos = libra_arm_->getCommandPosition(LIBRA_HEBI::Joint::kJ3);
        if (j3_pos <= 30) {
            camera_pos_[0] = (j3_pos <= 0) ? -j3_pos : 0;
        } else {
            camera_pos_[0] = 180 - j3_pos;
        }

        // 目的のカメラのパンアングルを取得
        // Retrieve desired camera pan angle
        if (camera_dir_[1] != 0) {
            camera_pos_[1] += camera_dir_[1] * 90.0 / (60.0 * 60.0);
            if ((camera_pos_[1] > camera_setpos_[1]) == (camera_dir_[1] == 1)) {
                camera_pos_[1] = camera_setpos_[1];
                camera_dir_[1] = 0;
            }
        }

        // 目的のカメラのチルト角度を取得
        // Retrieve desired camera tilt angle
        if (camera_dir_[2] != 0) {
            camera_pos_[2] += camera_dir_[2] * 90.0 / (60.0 * 60.0);
            if ((camera_pos_[2] > camera_setpos_[2]) == (camera_dir_[2] == 1)) {
                camera_pos_[2] = camera_setpos_[2];
                camera_dir_[2] = 0;
            }
        }

        // カメラサーボデータの更新
        // Update camera servo data
        for (int i = 0; i < 3; i++) {
            char str[20] = "";
            sprintf(str, "%8.2f deg", camera_pos_[i]);
            auto strW = GetDrawStringWidthToHandle(str, strlen(str), mainfont);
            // NOLINTNEXTLINE(readability-magic-numbers): Position of UI element
            DrawFormatStringToHandle(3670 - strW, 300 + 24 + 150 * i, maincolor,
                                     mainfont, str);
        }

        // SerialServoのArduinoにコマンドを送る
        // Send commands to SerialServo Arduino
        std::stringstream servo_str;
        servo_str << camera_pos_[0] << " " << camera_pos_[1] << " "
                  << camera_pos_[2] << "\n";
        ser_servo_->WriteStr(servo_str.str());  // send command

        /* ----- FLUID SYSTEM ----- */

        // SerialWaterのArduinoにコマンドを送る
        // Send commands to SerialWater Arduino
        switch (water_mode_) {
            case kStandby:  // 通常運転 - Normal operational mode
                water_cmd = 0;

                // トルク超過（5.0以上）
                // Excess torque (> 5.0)
                if (water_en_
                    && (abs(libra_arm_->getFeedbackEffortMA()) > 5.0
                        || abs(libra_arm_->getFeedbackEffortMB()) > 5.0)) {
                    // Pause arm movement
                    libra_arm_->stop();
                    water_mode_ = WaterMode::kAdjust;
                }
                break;

            case kAdjust:  // 水位調整 - Water level adjustment mode
                // トルクに通常反応（2.5～5.0）
                // Normal response to torque (2.5-5.0)
                if (water_en_
                    && (abs(libra_arm_->getFeedbackEffortMA()) >= 2.5
                        || abs(libra_arm_->getFeedbackEffortMB()) >= 2.5)) {
                    if (count == 0) {
                        double theta = atan2(libra_arm_->getFeedbackEffort(
                                                 LIBRA_HEBI::Joint::kPitch),
                                             libra_arm_->getFeedbackEffort(
                                                 LIBRA_HEBI::Joint::kRoll));

                        // A入 | B入 | A出 | B出 - A_IN | B_IN | A_OUT | B_OUT
                        if (theta > M_PI * 7 / 8
                            || -M_PI * 7 / 8 >= theta) {  // W
                            water_cmd = 0b1001;
                        } else if (theta > M_PI * 5 / 8) {  // NW
                            water_cmd = 0b0001;
                        } else if (theta > M_PI * 3 / 8) {  // N
                            water_cmd = 0b0011;
                        } else if (theta > M_PI * 1 / 8) {  // NE
                            water_cmd = 0b0010;
                        } else if (theta > -M_PI * 1 / 8) {  // E
                            water_cmd = 0b0110;
                        } else if (theta > -M_PI * 3 / 8) {  // SE
                            water_cmd = 0b0100;
                        } else if (theta > -M_PI * 5 / 8) {  // S
                            water_cmd = 0b1100;
                        } else {  // SW
                            water_cmd = 0b1000;
                        }
                    }
                }

                // トルクが戻った（2.5以下）
                // If torque subsides (< 2.5)
                else if (count == 0) {
                    // Put fluid system on standby
                    water_mode_ = WaterMode::kStandby;
                    water_cmd = 0;

                    // Resume arm movement
                    libra_arm_->move(input_[0], input_[1], input_[2], input_[3],
                                     input_[4]);
                }
                break;

            case kDrain:
                // Keep draining until ENABLE or DISABLE are clicked
                water_cmd = 0b0011;
                break;

            default:
                std::cerr << "[ERROR] Invalid water mode: " << water_mode_
                          << std::endl;
        }

        ser_water_->Write(water_cmd);  // send command

        // 流体システムの状態
        // Status of fluid system
        for (int i = 0; i < 4; i++) {
            // NOLINTBEGIN(readability-magic-numbers): Position of UI element

            if (!water_en_) {
                DrawFormatStringToHandle(800 + 300 * i, 300, maincolor,
                                         mainfont, "DISABLE");
            } else if (water_cmd & 1 << (3 - i)) {
                DrawFormatStringToHandle(800 + 300 * i, 300,
                                         i < 2 ? GetColor(50, 150, 50)
                                               : GetColor(0, 0, 255),
                                         titlefont, "ON");
            } else {
                DrawFormatStringToHandle(800 + 300 * i, 300, maincolor,
                                         mainfont, "OFF");
            }

            // NOLINTEND(readability-magic-numbers): Position of UI element
        }

        /* ----- LOGGING ----- */

        // 連続ログの更新
        // Update continuous log
        if (count == 0) {
            /*
            *continuous_log_ << "Time,,";
            *continuous_log_ <<
            "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
            *continuous_log_ <<
            "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
            *continuous_log_ <<
            "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,";
            *continuous_log_
            << "A_IN,B_IN,A_OUT,B_OUT,,";
            *continuous_log_ <<
            "TP_CamBase[deg],TP_CamPan[deg],TP_CamTilt[deg]";
            *continuous_log_ <<
            std::endl;
            */

            *continuous_log_ << GetDateTimeString() + ",,";
            for (int j = 0; j < 3; j++) {
                for (int i = 0; i < 5; i++) {
                    *continuous_log_ << value_[i][j];
                    *continuous_log_ << ",";
                }
                *continuous_log_ << ",";
            }
            for (int b = 0; b < 4; b++) {
                *continuous_log_ << ((water_cmd & (1 << (3 - b))) ? 1 : 0);
                *continuous_log_ << ",";
            }
            *continuous_log_ << ",";
            *continuous_log_ << camera_pos_[0] << "," << camera_pos_[1] << ","
                             << camera_pos_[2];
            *continuous_log_ << std::endl;
        }

        // TODO: needs comment
        count++;
        if (count == 30) {
            count = 0;
        }
    }
}

/**
 * @brief Windows callback for button click actions.
 */
void AppMgr::OnClick(View* view) {
    if (view == btn_enable_) {  // ENABLE
        water_en_ = true;
        water_mode_ = WaterMode::kStandby;

    } else if (view == btn_disable_) {  // DISABLE
        water_en_ = false;
        water_mode_ = WaterMode::kStandby;

    } else if (view == btn_drain_) {  // DRAIN
        water_en_ = true;
        water_mode_ = WaterMode::kDrain;

    } else if (view == btn_shot_) {  // SHOT LOG
        /*
        shot_log_ << "Time,,";
        shot_log_ <<
        "TP_Roll[deg],TP_Pitch[deg],TP_J1[deg],TP_J2[deg],TP_J3[deg],,";
        shot_log_ <<
        "PP_Roll[deg],PP_Pitch[deg],PP_J1[deg],PP_J2[deg],PP_J3[deg],,";
        shot_log_ << "PT_Roll[Nm],PT_Pitch[Nm],PT_J1[Nm],PT_J2[Nm],PT_J3[Nm],,";
        shot_log_ << "Voltage[V],Current[A]";
        */
        std::string dts = GetDateTimeString();
        *shot_log_ << dts + ",,";
        for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 5; i++) {
                *shot_log_ << value_[i][j];
                *shot_log_ << ",";
            }
            *shot_log_ << ",";
        }
        *shot_log_ << ibox_voltage_->GetNum();
        *shot_log_ << ",";
        *shot_log_ << ibox_current_->GetNum();
        *shot_log_ << std::endl;
        std::cout << "[INFO] Snapshot - " << dts
                  << " | Voltage: " << std::to_string(ibox_voltage_->GetNum())
                  << " V | Current: " << std::to_string(ibox_current_->GetNum())
                  << " A\n";

    } else if (view == btn_convert_) {  // CONVERT
        double r = ibox_r_->GetNum();
        double theta = ibox_theta_->GetNum();
        double L = 989;
        double L_hand = 1014;
        double a = L;
        double b = L + L_hand;

        double k = (r * r + a * a - b * b) / (2 * a);
        double alpha = (atan2(0, r) + atan2(sqrt(r * r - k * k), k));
        double beta = asin(r * sin(alpha) / b);

        ibox_j1_->SetNum(theta + alpha / M_PI * 180);
        ibox_j2_->SetNum(-180 + beta / M_PI * 180);
        ibox_j3_->SetNum(0);

    } else if (view == btn_start_) {  // START
        input_[0] = ibox_roll_->GetNum();
        input_[1] = ibox_pitch_->GetNum();
        input_[2] = ibox_j1_->GetNum();
        input_[3] = ibox_j2_->GetNum();
        input_[4] = ibox_j3_->GetNum();

        // 流体システムが作動していない時のみアームを動かす
        // Only move arm when fluid system isn't running
        if (water_mode_ == WaterMode::kStandby) {
            // Begin arm movement
            libra_arm_->move(input_[0], input_[1], input_[2], input_[3],
                             input_[4]);
        }

    } else if (view == btn_stop_) {  // STOP
        libra_arm_->stop();

    } else if (view == btn_up_) {  // R+
        ibox_r_->SetNum(ibox_r_->GetNum() + ibox_increment_->GetNum());
        OnClick(btn_convert_);

    } else if (view == btn_down_) {  // R-
        ibox_r_->SetNum(ibox_r_->GetNum() - ibox_increment_->GetNum());
        OnClick(btn_convert_);

    } else if (view == btn_left_) {  // Theta+
        ibox_theta_->SetNum(ibox_theta_->GetNum()
                            + ibox_increment_->GetNum() / ibox_r_->GetNum()
                                  * 180 / M_PI);
        OnClick(btn_convert_);

    } else if (view == btn_right_) {  // Theta-
        ibox_theta_->SetNum(ibox_theta_->GetNum()
                            - ibox_increment_->GetNum() / ibox_r_->GetNum()
                                  * 180 / M_PI);
        OnClick(btn_convert_);

    } else if (view == btn_servo_slow_) {  // SLOW
        camera_setpos_[1] = ibox_camera_pan_->GetNum();
        camera_setpos_[2] = ibox_camera_tilt_->GetNum();
        camera_dir_[1] = (ibox_camera_pan_->GetNum() >= camera_pos_[1]) ? 1
                                                                        : -1;
        camera_dir_[2] = (ibox_camera_tilt_->GetNum() >= camera_pos_[2]) ? 1
                                                                         : -1;

    } else if (view == btn_servo_fast_) {  // FAST
        camera_pos_[1] = ibox_camera_pan_->GetNum();
        camera_pos_[2] = ibox_camera_tilt_->GetNum();
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Initializes the application UI (via the DX library).
 */
void AppMgr::SetupIncludeDxlibInit() {
    // NOLINTBEGIN(readability-magic-numbers): UI initialization

    // ウインドウモードで起動 - Start in windowed mode
    ChangeWindowMode(TRUE);

    // 最大化ボタンが存在するウインドウモードに変更
    // Set to windowed mode with maximize button present
    SetWindowStyleMode(7);

    // 画面サイズを指定 - Specify screen size
    SetGraphMode(kWindowW, kWindowH, 32);

    // サイズ変更を可能にする - Allow resizing
    SetWindowSizeChangeEnableFlag(TRUE, TRUE);

    // ウインドウサイズを指定 - Specify window size
    int desktop_w{0};
    int desktop_h{0};
    GetDefaultState(&desktop_w, &desktop_h, nullptr);

    // 横長ディスプレイ - Landscape display
    if (static_cast<float>(desktop_w) / desktop_h
        > static_cast<float>(kWindowW) / kWindowH) {
        SetWindowSize(0.8 * desktop_h * (kWindowW / kWindowH), 0.8 * desktop_h);
    }
    // 縦長ディスプレイ - Portrait display
    else {
        SetWindowSize(0.8 * desktop_w, 0.8 * desktop_w * (kWindowH / kWindowW));
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

    // アンチエイリアス付き図形描画の準備を行う - Prepare to draw anti-aliased
    // shapes
    BeginAADraw();

    // NOLINTEND(readability-magic-numbers): UI initialization
}

/**
 * @brief TODO.
 */
int AppMgr::printComList(void) {
    HDEVINFO h_devinfo;
    DWORD member_index = 0;
    SP_DEVINFO_DATA data = {sizeof(SP_DEVINFO_DATA)};

    int max = 0;
    // デバイス情報セットを取得 - Get device information set
    h_devinfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, nullptr,
                                    nullptr,
                                    DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (!h_devinfo) {
        // デバイス情報セットが取得できなかった場合
        // If the device information set could not be obtained
        return 0;
    }

    data.cbSize = sizeof(data);

    // デバイスインターフェイスの取得 - Get device interface
    while (SetupDiEnumDeviceInfo(h_devinfo, max, &data)) {
        DWORD dataT;
        DWORD size;
        LPTSTR buf;

        // COMポート名の取得 - Obtain COM port name
        HKEY key = SetupDiOpenDevRegKey(h_devinfo, &data, DICS_FLAG_GLOBAL, 0,
                                        DIREG_DEV, KEY_QUERY_VALUE);
        if (key) {
            TCHAR name[256];
            DWORD type = 0;
            size = sizeof(name);
            RegQueryValueEx(key, _T("PortName"), nullptr, &type, (LPBYTE)name,
                            &size);
            _tprintf(_TEXT("%s"), name);
        }

        // デバイスの説明を取得 - Get device description
        size = 0;
        buf = nullptr;
        while (!SetupDiGetDeviceRegistryProperty(h_devinfo, &data,
                                                 SPDRP_DEVICEDESC, &dataT,
                                                 (PBYTE)buf, size, &size)) {
            if (GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
                if (buf) {
                    LocalFree(buf);
                }
                buf = (LPTSTR)LocalAlloc(LPTR, size * 2);
            } else {
                break;
            }
        }

        _tprintf(_TEXT("(%s)\n"), buf);
        if (buf) {
            LocalFree(buf);
        }
        ++max;
    }

    // デバイス情報セットを解放 - Release device information set
    SetupDiDestroyDeviceInfoList(h_devinfo);

    return max;
}

/**
 * @brief Provides a formatted string of the current date and time.
 *
 * @return std::string Formatted as "YYYY/MM/DD HH:MM:SS.SSS"
 */
std::string AppMgr::GetDateTimeString() {
    SYSTEMTIME st;
    char dt_char[100];

    GetLocalTime(&st);
    sprintf(dt_char, "%04d/%02d/%02d %02d:%02d:%02d.%03d", st.wYear, st.wMonth,
            st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

    return std::string(dt_char);
}
