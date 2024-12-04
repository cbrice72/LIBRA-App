# HEBI Actuators

This document lists the necessary steps to set up and use HEBI actuators in either a Windows or Linux (**WSL only**) environment.

- [Before You Start](#before-you-start)
    - [*Downloading the API*](#downloading-the-api)
    - [*Networking*](#networking)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
    - [*Can't Access Actuator*](#cant-access-actuator)
    - [*Resetting Misaligned Actuators*](#resetting-misaligned-actuators)
- [Optional Items](#optional-items)
    - [*Downloading and Building Examples*](#downloading-and-building-examples)

## Before You Start

### *Downloading the API*

Download the latest API `.zip` (Windows) or `.tar.gz` (Linux) from the [HEBI tools website](https://docs.hebi.us/tools.html#cpp-api).
Then, simply extract it to your directory of choice.

### *Networking*

> **_NOTE:_** If you are using WSL and connecting the actuators via USB adapter, you **must** set your networking mode to `mirrored` to be able to see the extra networking adapter. See [SETUP.md](../SETUP.md) "WSL" -> "Networking Mode" for more information.

It is recommended that you use HEBI actuators via IP addressing as Windows and Linux do not come with DHCP server capabilities by default.
The following instructions will help you set up your environment for IP addressing.

Before setting a static IP, ensure that the actuator is powered on and connected via Ethernet (an Ethernet-to-USB adapter also works).
Then, in Windows Settings, navigate to "Network & Internet" and find the actuator's network based on your connection method.
- **Ethernet (via LAN port)**: in the main window, it may show up as "Unidentified Network", "Ethernet2", or something similar.
- **USB Adapter (via USB port)**: go to "Advanced network settings" and find the network with your adapter's name under it. Click on it to expand, then click "View additional properties".

Under "IP assignment", click "Edit" and manually assign IPv4 settings according to the table below.

| Setting | Value | Notes |
|---|---|---|
| IP address | `10.11.12.xxx` | `xxx` can be anything (e.g., `2`) as long as it doesn't clash with the default actuator address (`10.11.12.13`) or other user-assigned actuator addresses |
| Subnet mask | `255.255.255.0` | If instead you see "Subnet prefix length", enter `24` |
| Gateway | `10.11.12.1` | Doesn't matter, but required |

## Usage

In your root `CMakeLists.txt`, simply use the the `add_subdirectory()` command to add the HEBI C++ project path (e.g., `${CMAKE_SOURCE_DIR}/thirdparty/hebi-cpp-3.11.1`).
Make sure you do this before you add any of your project source files.
Then, in your call to `target_link_libraries()`, add `hebic++` as a dependency.

When you build your CMake project, the HEBI C++ API will automatically be built for you.

For examples, see the [hebi-cpp-examples](https://github.com/HebiRobotics/hebi-cpp-examples/tree/master) git repo.

## Troubleshooting

### *Can't Access Actuator*

If the actuator does not show up (e.g., in HEBI Scope) or is inaccessible, follow the instructions [here](https://docs.hebi.us/core_concepts.html#connectionless-static-ip-reset) to reset it to the default IP address of `10.11.12.13`.

### *Resetting Misaligned Actuators*

1. Open the HEBI Scope app
2. Select the misaligned actuator
3. Go to Monitoring and set "Position" to 0
4. Manually straighten the arm
    - Go to the ground floor and align it (rotate the motors) by hand
5. Go to Dashboard -> Advanced
    1. Set "Position Offset" to the opposite value
    2. Set "Effort" to 0

## Optional Items

### *Downloading and Building Examples*

Clone the git repo at [https://github.com/HebiRobotics/hebi-cpp-examples/tree/master](https://github.com/HebiRobotics/hebi-cpp-examples/tree/master) and follow the instructions in the `README.md`. It is recommended that you download the HEBI C++ API manually (this is explained in the README).

> **_NOTE:_** it is not necessary to build the `kits/` directory, so if you are getting compile/build errors from files in that directory, feel free to remove it from `/projects/cmake/CMakeLists.txt`. To do this, open that `CMakeLists.txt` and delete all lines pertaining to kits CMake targets (as of 2024/12/04, lines 274-321).
