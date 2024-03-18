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
#include <sstream>
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
 * @brief Standard constructor - automatically opens the given device.
 * 
 * @param device_name The device to connect to (e.g., "/dev/ttyASM0")
 */
LibraLidar::LibraLidar(const std::string& device_name) {
    // --- Connection ---

    if (device_name.empty()) {
        std::cerr << "[ERROR] LIDAR - Given device_name is empty; cannot init "
                     "sensor without valid path (e.g., \"/dev/ttyASM0\")!"
                  << std::endl;
        return;
    }

    if (!urg_.open(device_name.c_str(), qrk::Urg_driver::Default_baudrate,
                   qrk::Urg_driver::Serial)) {
        std::cerr << "[ERROR] LIDAR - Urg_driver::open(" << device_name
                  << ") failed: " << urg_.what() << std::endl;
        return;
    }

    // Retrieve metadata before measurement mode is turned on
    product_type_ = urg_.product_type();
    firmware_version_ = urg_.firmware_version();
    serial_id_ = urg_.serial_id();

    // --- Settings ---

    // Limit scanning range to camera FOV (default: 270 deg)
    urg_.set_scanning_parameter(urg_.deg2step(-90), urg_.deg2step(+90),
                                0);  // 180 deg

    // Reset LIDAR timestamp to match current PC system time
#ifdef DEBUG
    std::cout << "[DEBUG] LIDAR - Timestamp before: ";
    PrintTimestamp();
#endif

    auto pc_time_stamp = std::chrono::time_point_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now())
                             .time_since_epoch()
                             .count();
    urg_.set_sensor_time_stamp(pc_time_stamp);

#ifdef DEBUG
    std::cout << "[DEBUG] LIDAR - Timestamp after: ";
    PrintTimestamp();
#endif

    // --- Measurement ---

    // Spin up the LIDAR and prepare to retrieve data
    urg_.start_measurement(qrk::Urg_driver::Distance,
                           qrk::Urg_driver::Infinity_times, 0);
}

/**
 * @brief Standard destructor.
 */
LibraLidar::~LibraLidar() {
    // Gracefully close sensor connection
    if (urg_.is_open()) {
        urg_.close();
    }
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

//------------------------------------------------------------------------------
// !Getters & Setters
//------------------------------------------------------------------------------

/**
 * @brief Gets LIDAR metadata (product type, firmware version, etc.).
 *
 * @return Newline-delimited string of metadata in the format "Category: Value"
 */
std::string LibraLidar::GetMetadata() {
    std::stringstream ret;

    if (urg_.is_open()) {
        ret << "Type:      " << product_type_ << "\n";
        ret << "Firmware:  " << firmware_version_ << "\n";
        ret << "Serial ID: " << serial_id_ << "\n";
    } else {
        ret << "Not connected!";
    }

    return ret.str();
}

/**
 * @brief TODO
 *
 * @return TODO
 *
 * @note The Hokuyo URG library is not entirely cross-platform.
 *       In this case, `urg.get_distance()` only accepts a reference to a
 *       vector of Windows `long` (Linux alternative would be `uint64_t`).
 */
std::vector<long> LibraLidar::GetData() {
    std::vector<long> data;

    if (!urg_.get_distance(data)) {
        std::cerr << "[ERROR] LIDAR - Urg_driver::get_distance() failed: "
                  << urg_.what() << std::endl;
        return {};
    }

    return data;
}
