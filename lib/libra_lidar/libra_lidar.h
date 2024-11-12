/******************************************************************************
 * @file   libra_lidar.h
 * @brief  Control code for LIBRA manipulator Hokuyo LIDAR; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
// Other Libraries' Headers
//   URG2D
#include "Urg_driver.h"

// Project Headers
//   (none)

/**
 * @brief TODO: documentation
 */
class LibraLidar {
  public:
    explicit LibraLidar(const std::string& device_name);
    ~LibraLidar();

    // --- Sensor Commands ---

    // --- Getters & Setters ---

    std::string GetMetadata();
    std::vector<long> GetData();

    void SetDebugMode(bool enabled);

  private:
    // --- Helper Functions ---

    void PrintTimestamp();

    // --- Data Members ---

    bool debug_mode_{false};

    qrk::Urg_driver urg_;

    std::string product_type_;
    std::string firmware_version_;
    std::string serial_id_;
};
