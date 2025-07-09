# LIBRA-I Usage

A collection of detailed instructions and lessons learned from operating the LIBRA robot.

- [Networking](#networking)
    - [*LIBRA-I HEBI Actuators*](#libra-i-hebi-actuators)
    - [*LIBRA-II HEBI Actuators*](#libra-ii-hebi-actuators)
- [Operation](#operation)
    - [*Using LIBRA-I*](#using-libra-i)
    - [*Using LIBRA-II (w/ ETA)*](#using-libra-ii-w-eta)
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

### *Using LIBRA-I*

1. **Ensure all power cables are plugged in**
    - TODO: explain which cables do what
2. **Ensure all USB and Ethernet cables are plugged into your computer**
    - TODO: explain which cables do what
3. **Run `libra_app_gui` and click "Yes" on the startup popup** to automatically connect all actuators and peripherals. If you click "No", you will have to individually connect each device via the menu bar at the top of the GUI.
    - Connecting to the HEBI actuators automatically commands them to hold their position
    - Connecting to the RealSense camera requires that the all-in-one ROS2 launch file be run first (see [README.md "(2) ROS2 Nodes"](./README.md#2-ros2-nodes))
4. **Remove tape supports** keeping the arm in place
    - Around the furthest two arm links (1st floor)
    - Around the suspension pipe and counterweight links (2nd floor)
5. Using the GUI, **slowly deploy LIBRA to a horizontal position** (see [docs/LIBRA-I_Unfolding_Angles.xlsx](./docs/LIBRA-I_Unfolding_Angles.xlsx))

### *Using LIBRA-II (w/ ETA)*

TODO

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
