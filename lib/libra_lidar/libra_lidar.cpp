/******************************************************************************
 * @file   libra_lidar.cpp
 * @brief  Control code for LIBRA manipulator Hokuyo LIDAR; implementation file.
 *
 * @author Christian Brice
 ******************************************************************************/

// Related Header
#include "libra_lidar.h"

// C++ Standard Library Headers
#include <chrono>
#include <iostream>
#include <sstream>

// Other Library Headers
#include "ticks.h"  // URG2D (Hokuyo)

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

    // Limit scanning range to camera FOV, or 180 deg (default: 270 deg)
    urg_.set_scanning_parameter(urg_.deg2step(-90), urg_.deg2step(+90), 0);

    // Sync LIDAR and PC timestamps
    if (debug_mode_) {
        std::cout << "[DEBUG] LIDAR - Timestamp before: ";
        PrintTimestamp();
    }

    auto pc_time_stamp = std::chrono::time_point_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now())
                             .time_since_epoch()
                             .count();
    urg_.set_sensor_time_stamp(pc_time_stamp);

    if (debug_mode_) {
        std::cout << "[DEBUG] LIDAR - Timestamp after:  ";
        PrintTimestamp();
    }

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
 * @brief TODO: documentation
 *
 * @return TODO
 *
 * @note The Hokuyo URG library is not entirely cross-platform.
 *       In this case, `urg.get_distance()` only accepts a reference to a
 *       vector of Windows `long` (Linux alternative would be `uint64_t`).
 */
std::vector<long> LibraLidar::GetData() {
    std::vector<long> data;
    long timestamp;

    if (!urg_.get_distance(data, &timestamp)) {
        std::cerr << "[ERROR] LIDAR - Urg_driver::get_distance() failed: "
                  << urg_.what() << std::endl;
        return {};
    }

    return data;  // TODO: return timestamp, if necessary
}

/**
 * @brief Sets whether or not verbose debug text is displayed
 *
 * @param true to enable, false to disable
 *
 * @note This setting only affects `LibraLidar` functions.
 */
void LibraLidar::SetDebugMode(bool enabled) {
    debug_mode_ = enabled;
}
