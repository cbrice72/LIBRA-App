/******************************************************************************
 * @file   app_mgr.h
 * @brief  Application manager header file.
 *
 * @author Yuto Goto, Christian Brice
 * @date   ???
 ******************************************************************************/

#pragma once

// Other Libraries' Headers
// Project Headers
#include "libra_hebi.h"

/**
 * @brief The .NET application manager.
 */
class AppMgr {
  public:
    void Main();

  private:
    /**
     * @brief Logical status of the fluid system.
     */
    enum WaterMode { kStandby = 0, kAdjust, kDrain };

    // --- Main Window ---

    void MainThread();

    // --- Helper Functions ---

    static std::string GetDateTimeString();

    // --- Data Members ---

    std::unique_ptr<LIBRA_HEBI> libra_arm_;
    std::unique_ptr<Serial> ser_water_;
    std::unique_ptr<Serial> ser_servo_;

    volatile bool flag_thread_end_{0};
    volatile bool flag_end_{0};

    std::ofstream continuous_log_;
    std::ofstream snapshot_log_;

    bool water_en_{true};
    WaterMode water_mode_{kStandby};

    /*
    double input_[5];
    double value_[5][3] = {0};
    double camera_pos_[3] = {0};
    double camera_setpos_[3] = {0};
    int camera_dir_[3] = {0};
    */
    std::array<double, 5> input_{0};
    std::array<std::array<double, 3>, 5> value_{0};
    std::array<double, 3> camera_pos_{0};
    std::array<double, 3> camera_setpos_{0};
    std::array<int, 3> camera_dir_{0};
};
