/******************************************************************************
 * @file   lidar_hokuyo.h
 * @brief  Control code for a Hokuyo (URG) LIDAR; header file.
 *
 * @author Christian Brice
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>

// Other Library Headers
#include "Urg_driver.h"  // URG2D (Hokuyo)

// Project Headers
//   (none)

/**
 * @brief TODO: documentation
 */
class LidarHokuyo {
  public:
    explicit LidarHokuyo(const std::string& device_name);
    ~LidarHokuyo();

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
