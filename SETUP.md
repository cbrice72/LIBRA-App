# Setup

This document lists the necessary steps to set up an Ubuntu 22.04 development environment for the LIBRA App. Please contact Christian Brice ([email](mailto:brice.c.67b9@m.isct.ac.jp)) with any questions or revision suggestions.

If you wish to set up a virtual Linux container on **Windows**, follow the instructions in [docs/WINDOWS_SETUP.md](./docs/WINDOWS_SETUP.md) before continuing.

- [Network](#network)
    - [*Proxy Setup*](#proxy-setup)
    - [*HEBI Actuators*](#hebi-actuators)
- [Required Software](#required-software)
    - [*System Packages*](#system-packages)
    - [*ROS2 and SLAM*](#ros2-and-slam)
    - [*EPOS (Maxon) Library*](#epos-maxon-library)
- [Recommended Applications](#recommended-applications)
    - [*VS Code*](#vs-code)
    - [*Qt Creator*](#qt-creator)
    - [*HEBI Scope*](#hebi-scope)
- [Troubleshooting](#troubleshooting)
    - [*VS Code: prompted to "unlock a keyring" on every launch*](#vs-code-prompted-to-unlock-a-keyring-on-every-launch)
    - [*Unable to run Qt Creator*](#unable-to-run-qt-creator)
    - [*Qt Creator: "No valid license available"*](#qt-creator-no-valid-license-available)
    - [*EPOS controllers not appearing in device list*](#epos-controllers-not-appearing-in-device-list)

## Network

### *Proxy Setup*

If you are not behind a proxy (or you don't know what it is), skip ahead to [*Required Software*](#required-software).

Open `/etc/apt/apt.conf` (requires sudo) and add the following line with your proxy details. Note that the address *must* include the leading "http://" (e.g., `http://proxy.noc.titech.ac.jp:3128`).

```txt
Acquire::http::Proxy "<address>:<port>";
```

#### **Configuring the Git Proxy**

If you already have [Git](https://git-scm.com/) installed, go ahead and configure its proxy now. If not, remember to do so after the `sudo apt install` step in the next section. Note that the address *must* include the leading "http://".

```bash
git config --global http.proxy <address>:<port>
```

### *HEBI Actuators*

The HEBI actuators used in the LIBRA prototypes have statically-assigned IP addresses. In order to communicate with them, you must first configure your PC's local network (see [docs/HEBI.md "Networking"](./docs/HEBI.md#networking)).

## Required Software

### *System Packages*

Update the APT package lists and ensure your system is up to date.

```bash
sudo apt update && sudo apt upgrade
```

Install the following packages via the terminal.

```bash
sudo apt install -y build-essential clang libclang-dev clang-format clang-tidy cmake cmake-format doxygen git libgl1-mesa-dev qt6-base-dev libxcb-cursor0 libxcb-cursor-dev
```

- `build-essential`: programs and libraries necessary for basic software development.
- `clang` & `libclang-dev`: C/C++ compiler, required by `clang-format` and `clang-tidy` ([link](https://clang.llvm.org/)).
- `clang-format`: clang-based C++ formatter ([link](https://clang.llvm.org/docs/ClangFormat.html)).
- `clang-tidy`: clang-based C++ linter ([link](https://clang.llvm.org/extra/clang-tidy/)).
- `cmake`: cross-platform C++ build tool ([link](https://cmake.org/)).
- `cmake-format`: CMake formatter ([link](https://github.com/cheshirekow/cmake_format)).
- `doxygen`: C++ documentation generator ([link](https://www.doxygen.nl/)).
- `git`: popular open-source version control system ([link](https://git-scm.com)).
- `libgl1-mesa-dev`: open-source graphics library, used by Qt ([link](https://www.mesa3d.org/)).
- `qt6-base-dev`: Qt development libraries ([link](https://packages.ubuntu.com/jammy/qt6-base-dev)).
- `libxcb-cursor0` & `libxcb-cursor-dev`: cursor-related convenience libraries, required by Qt ([link](https://gitlab.freedesktop.org/xorg/lib/libxcb-cursor)).

### *ROS2 and SLAM*

If you plan to use the app with ROS2 functionality enabled, further package installation and system setup is required. See [LIBRA-ROS2-Tools/SETUP.md](./LIBRA-ROS2-Tools/SETUP.md) and [LIBRA-ROS2-Tools/docs/SLAM.md "Before You Start"](./LIBRA-ROS2-Tools/docs/SLAM.md#before-you-start) before continuing.

### *EPOS (Maxon) Library*

**This is only required if you are building for the *LIBRA-II* system.** Otherwise, skip ahead to [Recommended Applications](#recommended-applications).

In order to link against the EPOS library during compilation (specifically, using the `-lEposCmd` flag), you must install the EPOS library files on your system.

1. Extract the EPOS Library archive **into a non-project directory** of your choice, then navigate to the extracted directory.

    ```bash
    cd thirdparty/epos-6.8.1.0/
    unzip EPOS-Linux-Library-En.zip -d ~/Downloads
    cd ~/Downloads/EPOS_Linux_Library
    ```

2. Run the install script (make sure you give it execution permissions).

    ```bash
    chmod +x install.sh && sudo ./install.sh
    ```

3. Once the install script finishes, you can delete the extracted files.

    ```bash
    cd .. && rm -rf EPOS_Linux_Library/
    ```

## Recommended Applications

### *VS Code*

To install, simply [download](https://code.visualstudio.com/download) and run the `.deb`.

```bash
sudo dpkg -i <package_name>
```

### *Qt Creator*

This app's GUI is built on the [Qt](https://doc.qt.io/) development framework.

First, apply for a [Qt educational license](https://www.qt.io/qt-educational-license#application) (make sure to select "Qt Edu for Developers"). Then go to your [Account Page](https://account.qt.io/s/) -> "Downloads" and download the "Unified Qt Installer X.X.X. for Linux". To run it, you must first give the `.run` file execution permissions.

- **Via the GUI** &ndash; Navigate to the installer via the file explorer and do the following.
    - Right click, "Properties" -> "Permissions" tab
    - Ensure "Allow executing file as program" is checked

- **Via the Terminal** &ndash; Execute the following. Remember to replace the bracketed text with your Qt installer version.

    ```bash
    chmod +x qt-unified-linux-x64-<ver>-online.run
    ./qt-unified-linux-x64-<ver>-online.run
    ```

> ***NOTE:*** If you're behind a proxy, open the settings menu (bottom left) and select "Manual proxy configuration". Enter your proxy settings **without** the preceding `http://` (e.g., HTTP proxy: `proxy.noc.titech.ac.jp` Port: `3128`).

Log in and follow the installation procedure. The following list of installation steps show non-default configurations that you **must** install.

1. "Installation options"
    - Select "Qt X.X for desktop development" and "Custom Installation".
2. "Customize"
    - Under "Qt" -> "Qt X.X.X" (whichever is automatically selected) -> "Additional Libraries", check the following.
        - While most libraries listed in the `set(QT_PACKAGES ...)` line in the root `CMakeLists.txt` are available by default, some must be manually selected. At the time of writing, these are: **Qt Multimedia** and **Qt Serial Port**.
    - Ensure "Qt Creator" -> "Debug Symbols" is checked.

#### **Add Qt Creator to PATH**

To run the GUI application from the terminal (via the command `qtcreator`), add the following to the end of your `~/.bashrc`.

```bash
if [ -d "$HOME/Qt/Tools" ]; then
    PATH="$PATH:$HOME/Qt/Tools/QtCreator/bin"
fi
```

### *HEBI Scope*

Scope is very useful for visualizing the state of HEBI actuators and adjusting their parameters in real time. You can download the [latest release](https://docs.hebi.us/downloads_changelogs.html#software) on the HEBI docs website.

Installation on Linux requires a couple extra steps on the command line.

```bash
# Install required dependencies
sudo apt install -y libgdk-pixbuf2.0-0
# Install Scope via dpkg
sudo dpkg -i hebi-robotics-scope_<ver>_<architecture>.deb
# Run via terminal
hebi-scope
```

<hr>

## Troubleshooting

### *VS Code: prompted to "unlock a keyring" on every launch*

1. Open your display manager config file. If you're not sure what that is, look for a file ending in "dm" in the `/etc/pam.d` directory (e.g., `sddm`, `lightdm`).

    ```bash
    sudo nano /etc/pam.d/<file ending in "dm">
    ```

2. Check if the following "keyring" lines exist. If they do, simply remove the preceding dashes (`-`). Otherwise, append them to the end of their respective sections as shown below.

    ```txt
    @include common-auth
    auth    optional        pam_gnome_keyring.so
    ...
    @include common-session
    session optional        pam_gnome_keyring.so auto_start
    ```

### *Unable to run Qt Creator*

If you are using a non-GNOME desktop environment (e.g., Windows's WSL2 or Linux Mint's "Cinnamon" environment) you may run into problems with the Qt Creator UI. This might be because you're missing some display-related packages that Qt expects. Try the following catch-all install command for X11 display server protocol libraries.

```bash
sudo apt install '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev
```

If Qt Creator still doesn't run, try explicitly installing the following packages.

```bash
sudo apt install -y libfontconfig libxcb-glx0 libx11-xcb1 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-shape0 libxcb-xkb1 libxcb-xinerama0 libxkbcommon-x11-0 libegl1
```

See this [Stack Overflow thread](https://stackoverflow.com/questions/68036484/qt6-qt-qpa-plugin-could-not-load-the-qt-platform-plugin-xcb-in-even-thou) for more details.

### *Qt Creator: "No valid license available"*

You might get this error upon opening Qt Creator for the first time. It can be fixed via the Qt Maintenance Tool.

1. Open Qt Maintenance Tool as superuser.

    ```bash
    sudo /opt/Qt/MaintenanceTool
    ```

2. Wait for the "Performing license check" message to give you the "Cancel" prompt, then cancel it.
3. Open the settings menu and select "Manual proxy configuration". Enter your proxy information, save, then exit the dialog.
    - Since Qt Maintenance Tool is technically a different app, the proxy settings you entered in Qt Installer may not be set correctly.
4. Click "Retry", then login once the initial check finishes.
5. Select "Update components", then click "Next".
    - Once the operation finishes, you may get a message saying that there are no further updates required. This is fine &ndash; your license info was still updated.
6. Close Qt Maintenance Tool.

You should now be able to use Qt Creator.

### *EPOS controllers not appearing in device list*

Although the driver should be automatically installed when an EPOS controller is connected to your PC (via USB) for the first time, this may occasionally fail. If the driver was *not* installed, the Windows Device Manager will show "Unknown device" with a warning icon.

Follow the instructions in [the EPOS USB Driver Installation PDF](thirdparty/epos-6.8.1.0/driver/EPOS%20USB%20Driver%20Installation.pdf). All driver installation files are provided in `thirdparty/epos-6.8.1.0/driver/`.
