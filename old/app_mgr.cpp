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
#include "pretty_print.h"
#include "serial.h"

/* --- TABLE OF CONTENTS ---
 * !Main Window
 * !Helper Functions
 */

constexpr int kWindowW = 1920 * 2;  // app window width
constexpr int kWindowH = 1080 * 2;  // app window height

constexpr int kHebiNodeCount = 5;      // total number of HEBI actuators
constexpr int kHebiFeedbackCount = 3;  // total number of actuator feedback types

constexpr int kMaxonNodeCount = 1;  // total number of Maxon (EPOS) actuators

constexpr int kFluidStateCount = 4;  // number of pumps * number of pump states

constexpr int kCameraNodeCount = 3;  // total number of camera servos

//------------------------------------------------------------------------------
// !Main Window
//------------------------------------------------------------------------------

/**
 * @brief Initializes the app, spins off threads, and tears down the
 * environment on exit.
 */
void AppMgr::Main() {
    /* ----- INITIALIZATION ----- */

    // clang-format off
    colorize::Print(  // note: do NOT mess with the spacing!
        "\n"
        " 888      8888888 888888b.   8888888b.         d8888             d8888 \n"
        " 888        888   888  \"88b  888   Y88b       d88888            d88888 \n"
        " 888        888   888  .88P  888    888      d88P888           d88P888 \n"
        " 888        888   8888888K.  888   d88P     d88P 888          d88P 888 88888b.  88888b. \n"
        " 888        888   888  \"Y88b 8888888P\"     d88P  888         d88P  888 888 \"88b 888 \"88b \n"
        " 888        888   888    888 888 T88b     d88P   888        d88P   888 888  888 888  888 \n"
        " 888        888   888   d88P 888  T88b   d8888888888       d8888888888 888 d88P 888 d88P \n"
        " 88888888 8888888 8888888P\"  888   T88b d88P     888      d88P     888 88888P\"  88888P\" \n"
        "                                                                       888      888 \n"
        "                                                                       888      888 \n"
        "                                                                       888      888 \n"
        "\n",
        colorize::Level::kTitle);
    // clang-format on

    std::string answer;  // used to retrieve user input via std::cin

    // HEBIアクチュエータを接続
    // Connect HEBI actuators
    libra_arm_ = std::make_unique<LIBRA_HEBI>();
    while (!libra_arm_->Connect()) {
        colorize::Print("Try again? [y/n]:\n", colorize::Level::kPrompt);
        std::cin >> answer;
        if (answer == "n") {
            // HEBIアクチュエータに接続できない場合は、プログラムを終了
            // Exit app if connection to HEBI actuators can't be established
            colorize::Print("Exiting...\n", colorize::Level::kInfo);
            Sleep(1000);  // give user time to read message
            return;
        }
    }

    /* ----- THREAD MANAGEMENT ----- */

    // ProcessMessage以外の処理を行うスレッドを作成
    // Create thread for any non-DxLib processing
    CreateThread(nullptr, 0, MainThread_dmy, this, 0, nullptr);

    // ProcessMessageループ
    // ProcessMessage loop
    while (ProcessMessage() == 0 && !flag_thread_end_) {
        // 少しCPUを休める - Wait for MainThread to finish
        Sleep(6);
    }

    /* ----- TEARDOWN ----- */

    // プログラムが終了したことを示すフラグを立てる
    // Flag to indicate that the program has finished
    flag_end_ = true;

    // スレッド終了フラグが立つまで待つ
    // Wait until the thread is flagged as closed
    while (!flag_thread_end_) {
        Sleep(10);
    }

    // ログの終了
    // Close the logging streams
    continuous_log_.close();
    snapshot_log_.close();

    // DXライブラリのクリーンアップ
    // Clean up DX library
    DxLib_End();
}

/**
 * @brief Runner for MainThread.
 */
DWORD WINAPI AppMgr::MainThread_dmy(LPVOID pv) {
    auto* p = (AppMgr*)pv;
    p->MainThread();  // loops until app is closed by user
    p->flag_thread_end_ = true;
    return 0;
}

/**
 * @brief The primary program control loop; initializes the UI and continuously
 * receives user inputs & updates sensor readings.
 */
void AppMgr::MainThread() {
    // InputBoxのデフォルト値を設定
    // Set InputBox default values
    ibox_roll_->SetNum(libra_arm_->GetCommandPosition(LIBRA_HEBI::Joint::kRoll));
    ibox_pitch_->SetNum(
        libra_arm_->GetCommandPosition(LIBRA_HEBI::Joint::kPitch));
    ibox_j1_->SetNum(libra_arm_->GetCommandPosition(LIBRA_HEBI::Joint::kJ1));
    ibox_j2_->SetNum(libra_arm_->GetCommandPosition(LIBRA_HEBI::Joint::kJ2));
    ibox_j3_->SetNum(libra_arm_->GetCommandPosition(LIBRA_HEBI::Joint::kJ3));
    ibox_r_->SetNum(1200);
    ibox_theta_->SetNum(0);
    ibox_increment_->SetNum(10);
    ibox_voltage_->SetNum(24);
    ibox_current_->SetNum(0);
    ibox_camera_pan_->SetNum(0);
    ibox_camera_tilt_->SetNum(0);

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
        colorize::Print("Created log directory\n", colorize::Level::kInfo);
    }

    continuous_log_.open("./log/" + dt_path_str + "_continuous_log.csv");
    continuous_log_
        << "Time,,"
        << "TP_Roll (deg),TP_Pitch (deg),TP_J1 (deg),TP_J2 (deg),TP_J3 (deg),,"
        << "PP_Roll (deg),PP_Pitch (deg),PP_J1 (deg),PP_J2 (deg),PP_J3 (deg),,"
        << "PT_Roll (Nm),PT_Pitch (Nm),PT_J1 (Nm),PT_J2 (Nm),PT_J3 (Nm),,"
        << "A_IN,B_IN,A_OUT,B_OUT,,"
        << "TP_CamBase (deg),TP_CamPan (deg),TP_CamTilt (deg)\n";

    snapshot_log_.open("./log/" + dt_path_str + "_shot_log.csv");
    snapshot_log_
        << "Time,,"
        << "TP_Roll (deg),TP_Pitch (deg),TP_J1 (deg),TP_J2 (deg),TP_J3 (deg),,"
        << "PP_Roll (deg),PP_Pitch (deg),PP_J1 (deg),PP_J2 (deg),PP_J3 (deg),,"
        << "PT_Roll (Nm),PT_Pitch (Nm),PT_J1 (Nm),PT_J2 (Nm),PT_J3 (Nm),,"
        << "Voltage (V),Current (A)\n";

    // 更新ループ
    // Update loop
    while (!flag_end_ && ScreenFlip() == 0 && ClearDrawScreen() == 0) {
        // HEBIアクチュエータのデータを取得
        // Retrieve HEBI actuator data
        for (auto i = 0; i < kHebiNodeCount; i++) {
            auto joint = static_cast<LIBRA_HEBI::Joint>(i);
            value_.at(i).at(0) = libra_arm_->GetCommandPosition(joint);
            value_.at(i).at(1) = libra_arm_->GetFeedbackPosition(joint);
            value_.at(i).at(2) = libra_arm_->GetFeedbackEffort(joint);
        }

        // アーム制御のラベルとアクチュエータデータ
        // Arm control labels and actuator data
        std::string menu[] = {"Roll", "Pitch", "J1", "J2", "J3"};
        for (auto i = 0; i < kHebiNodeCount; i++) {
            // Target Pos., Current Pos., Current Torque
            for (auto j = 0; j < kHebiFeedbackCount; j++) {
                char str[20];
                int strW;
                sprintf(str, "%8.2f %s", value_.at(i).at(j),
                        j != 2 ? "deg" : "Nm");
                strW = GetDrawStringWidthToHandle(str, strlen(str), main_font);
                DrawFormatStringToHandle(1750 + 400 * j - strW, 824 + 200 * i,
                                         main_color, main_font, str);
            }
        }

        // 重心グラフ表示
        // Center of mass visualization
        const int c_x = 3100;
        const int c_y = kWindowH / 2 + 350;

        DrawLineAA(c_x + 40 * 0, c_y - 40 * 10, c_x + 40 * (-10), c_y - 40 * 0,
                   main_color, 2.5);
        DrawLineAA(c_x + 40 * -10, c_y - 40 * 0, c_x + 40 * (0), c_y - 40 * -10,
                   main_color, 2.5);
        DrawLineAA(c_x + 40 * 0, c_y - 40 * -10, c_x + 40 * (10), c_y - 40 * 0,
                   main_color, 2.5);
        DrawLineAA(c_x + 40 * 10, c_y - 40 * 0, c_x + 40 * (0), c_y - 40 * 10,
                   main_color, 2.5);

        DrawLineAA(c_x + 40 * (0), c_y - 40 * (5), c_x + 40 * (-5),
                   c_y - 40 * (0), main_color, 2.5);
        DrawLineAA(c_x + 40 * (-5), c_y - 40 * (0), c_x + 40 * (0),
                   c_y - 40 * (-5), main_color, 2.5);
        DrawLineAA(c_x + 40 * (0), c_y - 40 * (-5), c_x + 40 * (5),
                   c_y - 40 * (0), main_color, 2.5);
        DrawLineAA(c_x + 40 * (5), c_y - 40 * (0), c_x + 40 * (0),
                   c_y - 40 * (5), main_color, 2.5);

        DrawLineAA(c_x - 500, c_y, c_x + 500, c_y, main_color, 2.5);
        DrawLineAA(c_x + 500 * cos(M_PI * 1 / 8), c_y + 500 * sin(M_PI * 1 / 8),
                   c_x - 500 * cos(M_PI * 1 / 8), c_y - 500 * sin(M_PI * 1 / 8),
                   main_color, 1);
        DrawLineAA(c_x + 500 * cos(M_PI * 3 / 8), c_y + 500 * sin(M_PI * 3 / 8),
                   c_x - 500 * cos(M_PI * 3 / 8), c_y - 500 * sin(M_PI * 3 / 8),
                   main_color, 1);
        DrawLineAA(c_x + 500 * cos(M_PI * 5 / 8), c_y + 500 * sin(M_PI * 5 / 8),
                   c_x - 500 * cos(M_PI * 5 / 8), c_y - 500 * sin(M_PI * 5 / 8),
                   main_color, 1);
        DrawLineAA(c_x + 500 * cos(M_PI * 7 / 8), c_y + 500 * sin(M_PI * 7 / 8),
                   c_x - 500 * cos(M_PI * 7 / 8), c_y - 500 * sin(M_PI * 7 / 8),
                   main_color, 1);
        DrawLineAA(c_x, c_y - 500, c_x, c_y + 500, main_color, 2.5);
        DrawTriangleAA(c_x + 500, c_y, c_x + 480, c_y + 10, c_x + 480, c_y - 10,
                       main_color, TRUE);
        DrawTriangleAA(c_x, c_y - 500, c_x + 10, c_y - 480, c_x - 10, c_y - 480,
                       main_color, TRUE);
        DrawCircleAA(c_x + value_.at(0).at(2) * 40,
                     c_y - value_.at(1).at(2) * 40, 15, 20, GetColor(0, 0, 0),
                     TRUE);
        DrawFormatStringToHandle(c_x + 525, c_y - 25, main_color, main_font,
                                 "Roll (Nm)");
        DrawFormatStringToHandle(c_x - 100, c_y - 550 - 25, main_color,
                                 main_font, "Pitch (Nm)");

        // J3を負にするピッチ角を計算
        // Calculate pitch angle to negate J3
        const double j3_pos = libra_arm_->GetCommandPosition(
            LIBRA_HEBI::Joint::kJ3);
        if (j3_pos <= 30) {
            camera_pos_.at(0) = (j3_pos <= 0) ? -j3_pos : 0;
        } else {
            camera_pos_.at(0) = 180 - j3_pos;
        }

        // 目的のカメラのパンアングルを取得
        // Retrieve desired camera pan angle
        if (camera_dir_.at(1) != 0) {
            camera_pos_.at(1) += camera_dir_.at(1) * 90.0 / (60.0 * 60.0);
            if ((camera_pos_.at(1) > camera_setpos_.at(1))
                == (camera_dir_.at(1) == 1)) {
                camera_pos_.at(1) = camera_setpos_.at(1);
                camera_dir_.at(1) = 0;
            }
        }

        // 目的のカメラのチルト角度を取得
        // Retrieve desired camera tilt angle
        if (camera_dir_.at(2) != 0) {
            camera_pos_.at(2) += camera_dir_.at(2) * 90.0 / (60.0 * 60.0);
            if ((camera_pos_.at(2) > camera_setpos_.at(2))
                == (camera_dir_.at(2) == 1)) {
                camera_pos_.at(2) = camera_setpos_.at(2);
                camera_dir_.at(2) = 0;
            }
        }

        // カメラサーボデータの更新
        // Update camera servo data
        for (auto i = 0; i < kCameraNodeCount; i++) {
            char str[20] = "";
            sprintf(str, "%8.2f deg", camera_pos_.at(i));
            auto strW = GetDrawStringWidthToHandle(str, strlen(str), main_font);
            // NOLINTNEXTLINE(readability-magic-numbers): Position of UI element
            DrawFormatStringToHandle(3670 - strW, 300 + 24 + 150 * i,
                                     main_color, main_font, str);
        }

        // SerialServoのArduinoにコマンドを送る
        // Send commands to SerialServo Arduino
        std::stringstream servo_str;
        servo_str << camera_pos_.at(0) << " " << camera_pos_.at(1) << " "
                  << camera_pos_.at(2) << "\n";
        ser_servo_->WriteStr(servo_str.str());  // send command

        /* ----- FLUID SYSTEM ----- */

        // SerialWaterのArduinoにコマンドを送る
        // Send commands to SerialWater Arduino
        switch (water_mode_) {
            case kStandby:  // 通常運転 - Normal operational mode
                water_cmd = 0;

                // トルク超過（5.0Nm以上）
                // Excess torque (> 5.0 Nm)
                if (water_en_  // TODO: refactor this
                    && (abs(libra_arm_->GetFeedbackEffortMA()) > 5.0
                        || abs(libra_arm_->GetFeedbackEffortMB()) > 5.0)) {
                    // Pause arm movement
                    libra_arm_->Stop();
                    water_mode_ = WaterMode::kAdjust;
                }
                break;

            case kAdjust:  // 水位調整 - Water level adjustment mode
                // トルクに通常反応（2.5～5.0Nm）
                // Normal response to torque (2.5-5.0 Nm)
                if (water_en_
                    && (abs(libra_arm_->GetFeedbackEffortMA()) >= 2.5
                        || abs(libra_arm_->GetFeedbackEffortMB()) >= 2.5)) {
                    if (count == 0) {
                        const double theta =
                            atan2(libra_arm_->GetFeedbackEffort(
                                      LIBRA_HEBI::Joint::kPitch),
                                  libra_arm_->GetFeedbackEffort(
                                      LIBRA_HEBI::Joint::kRoll));

                        // Bit field "0b1234" -> 1: A_IN | 2: B_IN | 3: A_OUT | 4: B_OUT
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

                // トルクが戻った（2.5Nm以下）
                // If torque subsides (< 2.5 Nm)
                else if (count == 0) {
                    // 液体システムをスタンバイ - Put fluid system on standby
                    water_mode_ = WaterMode::kStandby;
                    water_cmd = 0;

                    // アームの動きを再開 - Resume arm movement
                    libra_arm_->Move(input_.at(0), input_.at(1), input_.at(2),
                                     input_.at(3), input_.at(4));
                }
                break;

            case kDrain:
                // ENABLE・DISABLEがクリックされるまで排水
                // Drain until ENABLE or DISABLE are clicked
                water_cmd = 0b0011;

                // Pause arm movement
                libra_arm_->Stop();
                break;
        }

        ser_water_->Write(water_cmd);  // send command

        // 流体システムの状態
        // Status of fluid system
        for (auto i = 0; i < kFluidStateCount; i++) {
            // NOLINTBEGIN(readability-magic-numbers): Position of UI element

            if (!water_en_) {
                DrawFormatStringToHandle(800 + 300 * i, 300, main_color,
                                         main_font, "DISABLED");
            } else if (water_cmd & 1 << (3 - i)) {
                DrawFormatStringToHandle(800 + 300 * i, 300,
                                         i < 2 ? GetColor(50, 150, 50)
                                               : GetColor(0, 0, 255),
                                         title_font, "ON");
            } else {
                DrawFormatStringToHandle(800 + 300 * i, 300, main_color,
                                         main_font, "OFF");
            }

            // NOLINTEND(readability-magic-numbers): Position of UI element
        }

        /* ----- LOGGING ----- */

        // 連続ログの更新
        // Update continuous log
        if (count == 0) {
            // Timestamp
            continuous_log_ << GetDateTimeString() + ",,";

            // Actuator info
            for (auto i = 0; i < kHebiFeedbackCount; i++) {
                for (auto j = 0; j < kHebiNodeCount; j++) {
                    continuous_log_ << value_.at(j).at(i) << ",";
                }
                continuous_log_ << ",";
            }

            // Fluid system info
            for (auto i = 0; i < kFluidStateCount; i++) {
                continuous_log_ << ((water_cmd & (1 << (3 - i))) ? 1 : 0)
                                << ",";
            }
            continuous_log_ << ",";

            // Camera actuator info
            continuous_log_ << camera_pos_.at(0) << "," << camera_pos_.at(1)
                            << "," << camera_pos_.at(2) << "\n";
        }

        // TODO: what is this?
        count++;
        if (count == 30) {
            count = 0;
        }
    }
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Provides a formatted string of the current date and time.
 *
 * @return std::string Formatted as "YYYY/MM/DD HH:MM:SS.MSS"
 */
std::string AppMgr::GetDateTimeString() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count()
              % 1000;  // std::time doesn't give MS; we have to get it ourselves

    std::ostringstream dtss;
    dtss << std::put_time(&tm, "%Y/%m/%d %H:%M:%S.") << ms << "\n";

    return dtss.str();
}
