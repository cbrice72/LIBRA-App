# LIBRA-I Usage

A collection of detailed instructions and lessons learned from operating the LIBRA robot.

- [Extending the Arm in Kikura Lab (North Lab Bldg. 1)](#extending-the-arm-in-kikura-lab-north-lab-bldg-1)
    - [*LIBRA-I*](#libra-i)
    - [*LIBRA-II (w/ ETA)*](#libra-ii-w-eta)
- [Troubleshooting](#troubleshooting)
    - [*Networking with/Connecting to HEBI actuators*](#networking-withconnecting-to-hebi-actuators)
    - [*Qt-based apps crash on startup*](#qt-based-apps-crash-on-startup)

## Extending the Arm in Kikura Lab (North Lab Bldg. 1)

### *LIBRA-I*

(TODO: this will change once the Qt-based app is operational)

1. Plug in LIBRA power cables
2. Plug in LIBRA USB cable
3. Start `LIBRA_App`
    - This activates the HEBI motors so they can hold their position
4. Connect to serial ports/COMs via terminal (see [docs/Arduino_COM_IDs.txt](docs/Arduino_COM_IDs.txt))
5. Disable water system (click `DISABLE`)
6. Remove tape supports
    - Around the furthest two links (1st floor)
    - Around the mast and counterweight links (2nd floor)
7. Slowly unfold LIBRA (see [docs/LIBRA-I_Unfolding_Angles.xlsx](docs/LIBRA-I_Unfolding_Angles.xlsx))

### *LIBRA-II (w/ ETA)*

TODO

## Troubleshooting

### *Networking with/Connecting to HEBI actuators*

To connect to HEBI actuators without a DHCP server (i.e., router), a static IP must be assigned to each actuator as well as the computer running the control app. See [docs/HEBI.md](docs/HEBI.md) for more information.

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
