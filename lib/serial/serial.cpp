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
 * @brief Standard constructor.
 * 
 * @param name Arbitrary name given to
 * this `Serial` object (for debugging purposes)
 * @param port_num The serial
 * port number to connect to (e.g., the "0" in "/dev/ttyUSB0")
 */
Serial::Serial(std::string name, std::string port_num)
    : name_(name), port_("/dev/ttyUSB" + port_num) {}

/**
 * @brief Standard destructor.
 */
Serial::~Serial() {
    serial_.close();
}

//------------------------------------------------------------------------------
// !Public Functions
//------------------------------------------------------------------------------

/**
 * @brief TODO.
 *
 * @return int TODO
 */
int Serial::Open() {
    // Initialize port options
    unsigned int baud_rate = 115200;

    // Create I/O context
    boost::asio::io_context io_context;

    // Open serial port
    boost::asio::serial_port serial_(io_context, port_);

    // Set port options
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

    serial.open();

    if (!serial.is_open()) {
        std::cout << "[ERROR] Serial(" << name_ << ") - Could not open port "
                  << port_ << "\n";
        return -1;
    }

    return 0;
}

/**
 * @brief TODO.
 *
 * @param data TODO
 * @return int TODO
 */
int Serial::Write(uint8_t data) {
    boost::system::error_code error;
    std::size_t bytesWritten = boost::asio::write(serial,
                                                  boost::asio::buffer(data),
                                                  error);
    if (error) {
        std::cerr << "[ERROR] Serial(" << name_
                  << ") - Write failed. Error: " << error.message()
                  << std::endl;
        return -1;
    }

    return 0;
}

/**
 * @brief TODO.
 *
 * @param str TODO
 * @return int TODO
 */
int Serial::WriteStr(std::string str) {
    boost::system::error_code error;
    std::size_t bytesWritten = boost::asio::write(serial,
                                                  boost::asio::buffer(data),
                                                  error);
    if (error) {
        std::cerr << "[ERROR] Serial(" << name_
                  << ") - Write failed. Error: " << error.message()
                  << std::endl;
        return -1;
    }

    return 0;
}
