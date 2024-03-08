/******************************************************************************
 * @file   libra_lidar.h
 * @brief  Control code for LIBRA manipulator Hokuyo LIDAR; header file.
 *
 * @author Christian Brice
 * @date   2024/3/6
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
// Other Libraries' Headers
//   Hokuyo URG
#include "Urg_driver.h"
// Project Headers
//   (none)

/**
 * @brief TODO
 */
class LibraLidar {
  public:
    LibraLidar() = default;
    ~LibraLidar();

    // --- Sensor Commands ---

    void Open(const std::string& device_name);

    // --- Getters & Setters ---

    std::vector<long> GetData();

  private:
    // --- Helper Functions ---

    void PrintTimestamp();

    // --- Data Members ---

    qrk::Urg_driver urg_;
};
