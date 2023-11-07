/******************************************************************************
 * @file   AppMgr.h
 * @brief  Application manager header file.
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
//   Eigen
#include <Eigen/Sparse>
//   HEBI Actuators
#include <group_command.hpp>
#include <group_feedback.hpp>
#include <hebi.h>
// Project Headers
#include "Button.h"
#include "InputBox.h"
#include "LIBRA_HEBI.h"
#include "OnClickListener.h"
#include "Serial.h"

/**
 * @brief The .NET application manager.
 */
class AppMgr : public OnClickListener {
  public:
    void Main();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    // --- Main Window ---

    static DWORD WINAPI MainThread_dmy(LPVOID);
    void MainThread();
    void OnClick(View* view) override;

    // --- Helper Functions ---

    void SetupIncludeDxlibInit();
    int printComList();
    std::string GetDateTimeString();

    // --- UI: General ---

    Button* EnableButton;
    Button* DisableButton;

    Button* ShotButton;

    InputBox* InputBox_Voltage;
    InputBox* InputBox_Current;

    // --- UI: Goal Position ---

    Button* ConvertButton;
    Button* StartButton;
    Button* StopButton;

    Button* UpButton;     // R+
    Button* DownButton;   // R-
    Button* LeftButton;   // Theta+
    Button* RightButton;  // Theta-

    InputBox* InputBox_Roll;
    InputBox* InputBox_Pitch;
    InputBox* InputBox_J1;
    InputBox* InputBox_J2;
    InputBox* InputBox_J3;

    InputBox* InputBox_Increment;
    InputBox* InputBox_R;
    InputBox* InputBox_Theta;

    // --- UI: Camera Position ---

    Button* ServoSlowButton;
    Button* ServoFastButton;

    InputBox* InputBox_CamPan;
    InputBox* InputBox_CamTilt;

    // --- Data Members ---

    Serial* SerialWater;
    Serial* SerialServo;
    LIBRA_HEBI* ARM;

    std::ofstream* continuous_log;
    std::ofstream* shot_log;

    bool enable {true};
    int mode {0};
    volatile bool EndFlag {0};
    volatile bool ThreadEndFlag {0};

    double input[5];
    double value[5][3] = {0};
    double camera_pos[3] = {0};
    double camera_setpos[3] = {0};
    int camera_dir[3] = {0};

    const int WindowW = 1920 * 2;
    const int WindowH = 1080 * 2;
};
