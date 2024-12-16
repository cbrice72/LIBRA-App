# Hokuyo (URG) LIDAR

This document lists the necessary steps to set up and use a Hokuyo (URG) LIDAR in a Linux environment.

- [Before You Start](#before-you-start)
    - [*Enabling Port Access*](#enabling-port-access)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Optional Items](#optional-items)
    - [*Windows Driver*](#windows-driver)
    - [*Sample Code*](#sample-code)

## Before You Start

### *Enabling Port Access*

In order for the app to open ports on your behalf *without sudo*, you need to add yourself to the `dialout` system group.

```bash
sudo adduser $USER dialout
```

Then, reboot in order for this to take effect.

To check whether this worked, first try the `groups` command in the terminal.
If you see `dialout` listed, then try connecting to the port while the LIDAR is running.

```bash
od /dev/ttyACM0
```

If there is no output (i.e., only a blank line), then it worked.
If you get a permissions error, it hasn't taken effect yet.

## Usage

Include `Urg_driver.h` at the top of your C++ file. This is a useful wrapper around all URG objects.

Sample code demonstrating use of the URG API can be found at the end of this document ([jump to sample code](#sample-code)).

## Troubleshooting

(none)

## Optional Items

### *Windows Driver*

To test the sample URG library apps (available [here](https://sourceforge.net/projects/urgnetwork/files/urg_library/)), **which are Windows-only**, you'll need to install the URG driver for Windows. The `URG_USB_Driver.inf` setup file can be found in `thirdparty/urg-cpp-1.2.7/driver/`.

To install, simply right-click the file in Windows and select "Install".
Note that you may need to disable digital signature checking in Windows - see the [README in the driver directory](thirdparty/urg-cpp-1.2.7/driver/README.md) for instructions.
For further troubleshooting, see the [official Microsoft docs](https://learn.microsoft.com/en-us/windows-hardware/drivers/ifs/using-an-inf-file-to-install-a-file-system-filter-driver).

### *Sample Code*

```cpp
/******************************************************************************
 * @brief  Sandbox file to test code structures from the URG LIDAR library.
 *
 * @author brice.c.aa
 * @date   2024/3/5
 ******************************************************************************/

// Related Header
//   (none)

// C++ Standard Library Headers
#include <iostream>

// POSIX/Windows Library Headers
#include "windows.h"

// Other Library Headers
#include "ticks.h"       // Hokuyo
#include "Urg_driver.h"  // Hokuyo

// Project Headers
#include "Connection_information.h"  // see "urg_library-x.x.x/samples/cpp/"

using namespace qrk;

namespace {
/**
 * @brief Prints X-Y coordinates for all measurement points.
 */
void print_xy(const Urg_driver& urg, const std::vector<long>& data,
              long time_stamp) {
    long min_distance = urg.min_distance();
    long max_distance = urg.max_distance();
    size_t data_n = data.size();
    for (size_t i = 0; i < data_n; ++i) {
        long l = data[i];
        if ((l <= min_distance) || (l >= max_distance)) {
            continue;
        }

        double radian = urg.index2rad(i);
        long x = static_cast<long>(l * cos(radian));
        long y = static_cast<long>(l * sin(radian));
        std::cout << "[" << time_stamp << "]  X-Y coords (" << x << ", " << y
                  << ")" << std::endl;
    }
    std::cout << std::endl;
}

/**
 * @brief Prints PC and LIDAR timestamps (in ??).
 */
void print_timestamp(Urg_driver& urg) {
    urg.start_time_stamp_mode();

    SYSTEMTIME st, lt;
    GetSystemTime(&st);

    std::cout << "PC: ticks=" << ticks()
              << "  |  LIDAR: " << urg.get_sensor_time_stamp() << std::endl;

    urg.stop_time_stamp_mode();
}
}  // namespace

int main(int argc, char* argv[]) {
    Connection_information information(argc, argv);

    // --- Connection ---

    Urg_driver urg;
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        std::cout << "Urg_driver::open(): " << information.device_or_ip_name()
                  << ": " << urg.what() << std::endl;
        return 1;
    }

    std::cout << "Connected!"
              << "\n";
    std::cout << "  Type:      " << urg.product_type() << "\n";
    std::cout << "  Firmware:  " << urg.firmware_version() << "\n";
    std::cout << "  Serial ID: " << urg.serial_id() << "\n";
    std::cout << "  Status:    " << urg.status() << "\n";
    std::cout << "  State:     " << urg.state() << std::endl;

    // --- Settings ---

    char in;

    std::cout << "Limit scanning range to 180 deg? [y/n]" << std::endl;
    std::cin >> in;
    if (in == 'y') {
        urg.set_scanning_parameter(urg.deg2step(-90), urg.deg2step(+90), 0);
    }

    std::cout << "Reset LIDAR timestamp? [y/n]" << std::endl;
    std::cin >> in;
    if (in == 'y') {
        std::cout << "Before -- ";
        print_timestamp(urg);

        // Configure current PC system time into the sensor
        urg.set_sensor_time_stamp(ticks());

        std::cout << "After  -- ";
        print_timestamp(urg);
    }

    // --- Measurements ---

    std::cout << "Enter any key to begin scan..." << std::endl;
    std::cin >> in;
    in = ' ';  // reset input for upcoming while loop

    urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
    while (in != 'y') {
        std::vector<long> data;
        long time_stamp = 0;

        if (!urg.get_distance(data, &time_stamp)) {
            std::cout << "Urg_driver::get_distance(): " << urg.what()
                      << std::endl;
            return 1;
        }
        print_xy(urg, data, time_stamp);

        std::cout << "End scan? [y/n]" << std::endl;
        std::cin >> in;
    }

#if defined(URG_MSC)
    getchar();
#endif

    // --- Cleanup ---

    urg.close();

    return 0;
}
```
