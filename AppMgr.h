#pragma once

#include "OnClickListener.h"
#include "Button.h"
#include "InputBox.h"
#include "Serial.h"
#include "LIBRA_HEBI.h"
#include <Eigen/Sparse>
#include <hebi.h>
#include <group_command.hpp>
#include <group_feedback.hpp>

class AppMgr : public OnClickListener {
private:
    Button* StopButton;
    Button* StartButton;
    Button* ConvertButton;
    Button* UpButton;
    Button* DownButton;
    Button* LeftButton;
    Button* RightButton;
    Button* EnableButton;
    Button* DisableButton;
    Button* ShotButton;
    Button* ServoSlowButton;
    Button* ServoFastButton;
    InputBox* InputBox_Roll;
    InputBox* InputBox_Pitch;
    InputBox* InputBox_J1;
    InputBox* InputBox_J2;
    InputBox* InputBox_J3;
    InputBox* InputBox_R;
    InputBox* InputBox_Theta;
    InputBox* InputBox_Increment;
    InputBox* InputBox_Voltage;
    InputBox* InputBox_Current;
    InputBox* InputBox_CamPan;
    InputBox* InputBox_CamTilt;
    Serial* SerialWater;
    Serial* SerialServo;
    LIBRA_HEBI* ARM;
    std::ofstream* shot_log;
    std::ofstream* continuous_log;

    volatile bool EndFlag = 0;
    volatile bool ThreadEndFlag = 0;
    double input[5];
    double value[5][3] = { 0 };
    double camera_pos[3] = {0};
    double camera_setpos[3] = { 0 };
    int camera_dir[3] = { 0 };
    int mode = 0;
    bool enable = true;

    const int WindowW = 1920 * 2;
    const int WindowH = 1080 * 2;

    void OnClick(View* view)override;
    static DWORD WINAPI MainThread_dmy(LPVOID);
    void MainThread();
    void SetupIncludeDxlibInit();
    std::string GetDateTimeString();
    int printComList();
    
public:
    void Main();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
