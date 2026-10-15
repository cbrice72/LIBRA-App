# LIBRA-I Usage

A collection of detailed instructions and lessons learned from operating the LIBRA robot.

- [Networking](#networking)
    - [*LIBRA-I HEBI Actuators*](#libra-i-hebi-actuators)
    - [*LIBRA-II HEBI Actuators*](#libra-ii-hebi-actuators)
- [Operation](#operation)
    - [*1. Power Everything On*](#1-power-everything-on)
    - [*2. Connect all USBs + Ethernet to Computer*](#2-connect-all-usbs--ethernet-to-computer)
    - [*3. Ensure You Have the Correct LIBRA App Version*](#3-ensure-you-have-the-correct-libra-app-version)
    - [*4. Run the LIBRA App and Set Up Devices*](#4-run-the-libra-app-and-set-up-devices)
    - [*5. Remove the straps*](#5-remove-the-straps)
    - [*6. Deploy LIBRA-I/II to a Horizontal Position*](#6-deploy-libra-iii-to-a-horizontal-position)
- [Troubleshooting](#troubleshooting)
    - [*Networking with/Connecting to HEBI actuators*](#networking-withconnecting-to-hebi-actuators)
    - [*Qt-based apps crash on startup*](#qt-based-apps-crash-on-startup)

## Networking

Communication with the HEBI X8-16 actuators used by both LIBRA prototypes is carried out via IP addressing. Each actuator's IP address is statically assigned (see the [official HEBI documentation](https://docs.hebi.us/core_concepts.html#static-address-assignment) for more details), and you must ensure no conflicting addresses exist on the network &ndash; **otherwise the network may become unusable!** Each actuator's IP address is listed below for reference.

The recommended IP address for your PC is `10.11.12.2` (see [docs/HEBI.md](./docs/HEBI.md) for instructions).

### *LIBRA-I HEBI Actuators*

| Actuator<br>Family | Actuator<br>Name | IP Address |
|---|---|---|
| LIBRA | MA | `10.11.12.100` |
| LIBRA | MB | `10.11.12.101` |
| LIBRA | J1 | `10.11.12.102` |
| LIBRA | J2 | `10.11.12.103` |
| LIBRA | J3 | `10.11.12.104` |

### *LIBRA-II HEBI Actuators*

| Actuator<br>Family | Actuator<br>Name | IP Address |
|---|---|---|
| LIBRA | Pitch | `10.11.12.100` |

## Operation

### *1. Power Everything On*

TODO

| Cable | *LIBRA-I* | *LIBRA-II* |
|---|---|---|
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |

### *2. Connect all USBs + Ethernet to Computer*

TODO

| Cable | *LIBRA-I* | *LIBRA-II* |
|---|---|---|
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |

### *3. Ensure You Have the Correct LIBRA App Version*

If you already have an executable, run it and confirm whether its `LIBRA_VERSION` number matches the number of the robot you're operating (e.g., `1` for LIBRA-I). If it doesn't, change `LIBRA_VERSION` via CMake options and rebuild the app (see [README.md "Building the App"](./README.md#building-the-app)).

### *4. Run the LIBRA App and Set Up Devices*

The app will automatically connect to everything if you click "Yes" on the startup popup. Otherwise, you'll have to individually connect the device(s) you plan on using via the menu bar.

Some things to note:

- Connecting to any actuators automatically commands them to hold their position.
- Connecting to the RGB-D camera and LiDAR requires that the all-in-one ROS2 launch file be run first (see [README.md "(2) ROS2 Nodes"](./README.md#2-ros2-nodes))

**Before moving onto the next step, command the actuators to their current position.** This puts them in position holding mode.

### *5. Remove the straps*

These hold the arm and counterweight(s) in place when the robot is powered off.

### *6. Deploy LIBRA-I/II to a Horizontal Position*

Using the app, adjust the robot's pitch until it is level with the ground (for LIBRA-I, see [docs/LIBRA-I_Unfolding_Angles.xlsx](./docs/LIBRA-I_Unfolding_Angles.xlsx)). When pitch torque feedback becomes too high, turn on automatic torque control (ATC) or manually adjust the counterweights until the system is in equilibrium.

The robot is now ready to use!

<hr>

## Troubleshooting

### *Networking with/Connecting to HEBI actuators*

To connect to HEBI actuators without a DHCP server (i.e., router), a static IP must be assigned to each actuator as well as the computer running the control app. See [docs/HEBI.md](./docs/HEBI.md) for more information.

### *Qt-based apps crash on startup*

Your graphics drivers might be outdated or incompatible with the versions of the software we're using. Particularly, **Qt-based GUI applications will suffer from frustrating crashes if your graphics drivers aren't up to date**. Follow the instructions below to add the right [Mesa](https://www.mesa3d.org/) package repository (PPA) and upgrade your drivers.

1. Check your current Mesa/OpenGL version using `glxinfo`. Save the output in case you have to revert the update later.

    ```bash
    sudo apt install mesa-utils
    glxinfo | grep "OpenGL version"
    ```

2. Add the appropriate PPA for updating your Mesa/OpenGL drivers.
    > ***NOTE:*** If you're on Ubuntu 24.04 "Noble" you can pick either PPA (although the second one, `oibaf/graphics-drivers`, is recommended).
    - For Ubuntu 18.04 "Bionic" to 24.04 "Noble":

        ```bash
        sudo add-apt-repository ppa:kisak/kisak-mesa -y
        ```

    - For Ubuntu 24.04 "Noble" to 25.04 "Plucky":

        ```bash
        sudo add-apt-repository ppa:oibaf/graphics-drivers -y
        ```

3. Update your graphics libraries (note: this may also update other graphics-adjacent packages, such as OpenCV).

    ```bash
    sudo apt update && sudo apt upgrade
    ```

4. Check your updated Mesa/OpenGl version once again. If it shows a newer version, try running `rviz2` by itself now. On the other hand, if you are suddenly experiencing other/new graphical issues, revert to the previous version.

    ```bash
    glxinfo | grep "OpenGL version"
    ```

For reference, see [this guide](https://linuxcapable.com/how-to-upgrade-mesa-drivers-on-ubuntu-linux/) on LinuxCapable.com and [this discussion](https://www.reddit.com/r/ROS/comments/11wh0m3/wsl2_does_not_display_gazebo_and_rviz/) on the r/ROS subreddit.
