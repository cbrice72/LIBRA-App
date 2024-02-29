/******************************************************************************
 * @file   serial.cpp
 * @brief  Control code for Arduinos via serial USB; implementation file.
 *         (adapted from Yuto Goto's work)
 *
 * @author Christian Brice
 * @date   2024/2/22
 ******************************************************************************/

// Related Header
#include "serial.h"
// C++ Standard Library Headers
#include <iostream>

// Other Libraries' Headers
//   (none)
// Project Headers
//   (none)

/* --- TABLE OF CONTENTS ---
 * !Public Functions
 */

/**
 * @brief Standard constructor - automatically opens the given serial port.
 *
 * @param name Arbitrary name given to this `Serial` object (for debugging
 *             purposes)
 * @param port The serial port to connect to (e.g., "/dev/ttyUSB0")
 */
Serial::Serial(std::string name, std::string port)
    : name_(name), port_(port), serial_(io_) {
    // Set port options
    unsigned int baud_rate = 115200;

    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

    // Open serial port
    serial_.open(port_);
}

/**
 * @brief Standard destructor - automatically closes the contained serial port.
 */
Serial::~Serial() {
    // Close serial port
    if (serial_.is_open()) {
        serial_.close();
    }
}

//------------------------------------------------------------------------------
// !Public Functions
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 *
 * @param data TODO
 * @return int TODO
 */
int Serial::Write(uint8_t data) {
    if (!serial_.is_open()) {
        std::cout << "[ERROR] Serial(" << name_
                  << ") - Failed to communicate with port " << port_ << "\n";
        return -1;
    }

    boost::system::error_code error;
    std::size_t bytesWritten = boost::asio::write(serial_,
                                                  boost::asio::buffer(&data, 1),
                                                  error);

    // The following is only necessary when using `serial_.write_some()` or
    // `serial_.async_write_some()`, as boost::asio::write() blocks the current
    // thread until all data has been received by the port (or an error is thrown)
    /*
    if (error) {
        std::cerr << "[ERROR] Serial(" << name_
                  << ") - Write failed. Error: " << error.message()
                  << std::endl;
        return -1;
    }
    */

    return 0;
}

/**
 * @brief TODO.
 *
 * @param str TODO
 * @return int TODO
 */
int Serial::WriteStr(std::string str) {
    if (!serial_.is_open()) {
        std::cout << "[ERROR] Serial(" << name_
                  << ") - Failed to communicate with port " << port_ << "\n";
        return -1;
    }

    boost::system::error_code error;
    std::size_t bytesWritten = boost::asio::write(serial_,
                                                  boost::asio::buffer(str),
                                                  error);

    // The following is only necessary when using `serial_.write_some()` or
    // `serial_.async_write_some()`, as boost::asio::write() blocks the current
    // thread until all data has been received by the port (or an error is thrown)
    /*
    if (error) {
        std::cerr << "[ERROR] Serial(" << name_
                  << ") - Write failed. Error: " << error.message()
                  << std::endl;
        return -1;
    }
    */

    return 0;
}
