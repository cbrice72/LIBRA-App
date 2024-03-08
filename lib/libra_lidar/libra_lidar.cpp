/******************************************************************************
 * @file   libra_lidar.cpp
 * @brief  Control code for LIBRA manipulator Hokuyo LIDAR; implementation file.
 *
 * @author Christian Brice
 * @date   2024/3/6
 ******************************************************************************/

// Related Header
#include "libra_lidar.h"
// C++ Standard Library Headers
#include <chrono>
#include <iostream>
// Other Libraries' Headers
//   Hokuyo URG
#include "ticks.h"

// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Helper Functions
 * !Sensor Commands
 * !Getters & Setters
 */

/**
 * @brief Standard destructor.
 */
LibraLidar::~LibraLidar() {
    // Properly terminate connection with sensor
    urg_.close();
}

//------------------------------------------------------------------------------
// !Helper Functions
//------------------------------------------------------------------------------

/**
 * @brief Prints current PC and LIDAR timestamps (in ms), for comparison
 */
void LibraLidar::PrintTimestamp() {
    urg_.start_time_stamp_mode();

    auto pc_time_stamp = std::chrono::time_point_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now())
                             .time_since_epoch()
                             .count();
    std::cout << "PC: " << pc_time_stamp
              << "  |  LIDAR: " << urg_.get_sensor_time_stamp() << std::endl;

    urg_.stop_time_stamp_mode();
}

//------------------------------------------------------------------------------
// !Sensor Commands
//------------------------------------------------------------------------------

void LibraLidar::Open(const std::string& device_name) {
    // --- Connection ---

    if (!urg_.open(device_name.c_str(), qrk::Urg_driver::Default_baudrate,
                   qrk::Urg_driver::Serial)) {
        std::cout << "[ERROR] LIDAR - Urg_driver::open(device_name) failed: "
                  << urg_.what() << std::endl;
        return;
    }

#ifdef DEBUG
    std::cout << "Connected!"
              << "\n";
    std::cout << "  Type:      " << urg_.product_type() << "\n";
    std::cout << "  Firmware:  " << urg_.firmware_version() << "\n";
    std::cout << "  Serial ID: " << urg_.serial_id() << "\n";
    std::cout << "  Status:    " << urg_.status() << "\n";
    std::cout << "  State:     " << urg_.state() << std::endl;
#endif

    // --- Settings ---

    // Limit scanning range to camera FOV (default: 270 deg)
    urg_.set_scanning_parameter(urg_.deg2step(-90), urg_.deg2step(+90),
                                0);  // 180 deg

    // Reset LIDAR timestamp to match current PC system time
#ifdef DEBUG
    std::cout << "[INFO] LIDAR - Timestamp before: ";
    PrintTimestamp();
#endif

    urg_.set_sensor_time_stamp(qrk::ticks());

#ifdef DEBUG
    std::cout << "[INFO] LIDAR - Timestamp after: ";
    PrintTimestamp();
#endif

    // --- Measurement ---

    // Spin up the LIDAR and prepare to retrieve data
    urg_.start_measurement(qrk::Urg_driver::Distance,
                           qrk::Urg_driver::Infinity_times, 0);
}

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief LibraLidar::GetData
 *
 * @return
 *
 * @note The Hokuyo URG library is not entirely cross-platform.
 *       In this case, `urg.get_distance()` only accepts a reference to a
 *       vector of Windows `long` (Linux alttern would be `uint64_t`).
 */
std::vector<long> LibraLidar::GetData() {
    std::vector<long> data;

    if (!urg_.get_distance(data)) {
        std::cout << "[ERROR] LIDAR - Urg_driver::get_distance() failed: "
                  << urg_.what() << std::endl;
        return std::vector<long>();  // empty vector
    }

    return data;
}
