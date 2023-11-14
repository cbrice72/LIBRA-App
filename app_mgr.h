/******************************************************************************
 * @file   app_mgr.h
 * @brief  Application manager header file.
 *
 * @author Yuto Goto, Christian Brice
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
#include "button.h"
#include "input_box.h"
#include "libra_hebi.h"
#include "onclick_listener.h"
#include "serial.h"

/**
 * @brief The .NET application manager.
 */
class AppMgr : public OnClickListener {
  public:
    void Main();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /**
     * @brief Logical status of the fluid system.
     */
    enum WaterMode { kStandby = 0, kAdjust, kDrain };

    // --- Main Window ---

    static DWORD WINAPI MainThread_dmy(LPVOID);

    void MainThread();
    void OnClick(View* view) override;

    // --- Helper Functions ---

    void SetupIncludeDxlibInit();
    static int printComList();
    static std::string GetDateTimeString();

    // --- UI: General ---

    Button* btn_enable_;
    Button* btn_disable_;
    Button* btn_drain_;

    Button* btn_shot_;

    InputBox* ibox_voltage_;
    InputBox* ibox_current_;

    // --- UI: Arm Position ---

    Button* btn_convert_;
    Button* btn_start_;
    Button* btn_stop_;

    Button* btn_up_;     // R+
    Button* btn_down_;   // R-
    Button* btn_left_;   // Theta+
    Button* btn_right_;  // Theta-

    InputBox* ibox_roll_;
    InputBox* ibox_pitch_;
    InputBox* ibox_j1_;
    InputBox* ibox_j2_;
    InputBox* ibox_j3_;

    InputBox* ibox_increment_;
    InputBox* ibox_r_;
    InputBox* ibox_theta_;

    // --- UI: Camera Position ---

    Button* btn_servo_slow_;
    Button* btn_servo_fast_;

    InputBox* ibox_camera_pan_;
    InputBox* ibox_camera_tilt_;

    // --- Data Members ---

    LIBRA_HEBI* libra_arm_;
    Serial* ser_water_;
    Serial* ser_servo_;

    volatile bool flag_thread_end_{0};
    volatile bool flag_end_{0};

    std::ofstream* continuous_log_;
    std::ofstream* shot_log_;

    bool water_en_{true};
    WaterMode water_mode_{kStandby};

    double input_[5];
    double value_[5][3] = {0};
    double camera_pos_[3] = {0};
    double camera_setpos_[3] = {0};
    int camera_dir_[3] = {0};
};
