/******************************************************************************
 * @file   serial.h
 * @brief  Control code for Arduinos via serial USB; header file.
 *         (adapted from Yuto Goto's work)
 *
 * @author Christian Brice
 * @date   2024/2/22
 ******************************************************************************/

#pragma once

// C++ Standard Library Headers
#include <string>
// Other Libraries' Headers
#include <boost/asio.hpp>

// Project Headers
//   (none)

/**
 * @brief TODO.
 *
 * @note For more information about Boost.Asio, see the boost documentation:
 *       https://www.boost.org/doc/libs/1_76_0/doc/html/boost_asio.html
 */
class Serial {
  public:
    explicit Serial(std::string name, std::string port);
    ~Serial();

    // --- Getters & Setters ---

    void SetDebugMode(bool enabled);
    int Write(uint8_t data);  // TODO: needed?
    int WriteStr(std::string str);

  private:
    // --- Data Members ---

    bool debug_mode_{false};

    std::string name_;
    std::string port_;

    boost::asio::io_context io_;  // only necessary for initializing serial port
    boost::asio::serial_port serial_;
};
