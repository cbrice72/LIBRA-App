# Setup

This document lists the necessary steps to set up an Ubuntu 22.04 development environment for the LIBRA App on a Windows 11 machine.
Please contact Christian Brice ([email](mailto:brice.c.aa@m.titech.ac.jp)) with any questions or revision suggestions.

## Table of Contents

1. [Setting Up](#setting-up)
    - [VM](#vm)
    - [WSL](#wsl)
2. [Preparing Your Development Environment](#preparing-your-development-environment)
    - [Proxy Settings](#proxy-settings)
    - [System Updates and Required Packages](#system-updates-and-required-packages)
    - [Required Drivers](#required-drivers)
3. [Project Software](#project-software)
    - [Qt Creator](#qt-creator)
    - [Maxon EPOS Library (system-wide install)](#maxon-epos-library-system-wide-install)
4. [Optional Items](#optional-items)

## Setting Up

You may set up a development environment in either a Virtual Machine (VM) or Windows Subsystem for Linux (WSL). There are a few differences you should be aware of.

| | Ability | Environment | USB Support | Shared Folder Support
|---|---|---|---|---|
| **VM** | Full-featured | Runs in separate environment | Select PC or VM on plug-in | Non-native; enable in VMWare settings + install [open-vm-tools](https://kb.vmware.com/s/article/2073803) |
| **WSL** | Lightweight | Runs natively in Windows | Non-native; install [USBIPD](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) + use every time |  Windows `C:\` drive located at `/mnt/c` |

### *VM*

1. Download and install [VMware Workstation Player](https://www.vmware.com/products/workstation-player.html).
    - **Version 17.5 is currently bugged!! Install Version 17.0.2 or lower**
    - If you use a non-English language keyboard, install the optional "Enhanced Keyboard Driver".
    - You may also use [VirtualBox](https://www.virtualbox.org/).
3. Download the [Ubuntu 22.04](https://ubuntu.com/download/desktop) OS image.
4. Open VMware and create a new Ubuntu virtual machine using the .iso you downloaded.
    - Most settings can be left as default, but you may want to allocate more space to the virtual hard disk (e.g., 80 GB).
5. Step through the Ubuntu installation once the VM initializes.
    - Note: "Erase disk and install Ubuntu" is referring to the virtual hard disk created by VMware, NOT your computer's actual hard disk.

I also recommend you allocate more cores and RAM to your VM; this can be done in Virtual Machine Settings.

- Memory: 8192 MB (8 GB)
- Processors: 4 cores

### *WSL*

1. Open a PowerShell terminal and install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install).
```bash
wsl --install -d Ubuntu-22.04
```

> **_NOTE:_** If you are getting the error "System Integrity policy has been violated", you will need to disable Smart App Control in Windows settings.

2. Once the installation is finished, enter the username and password you want to use when logging into the Ubuntu shell.

## Preparing Your Development Environment

### *Proxy Settings*

Open `/etc/apt/apt.conf` (requires sudo) and add the following line with your proxy details.
Note that the address *must* include the leading "http://" (e.g., `http://proxy.noc.titech.ac.jp:3128`).
```txt
Acquire::http::Proxy "[address]:[port]";
```

#### **Git**

If you already have Git installed, go ahead and configure its proxy now.
If not, remember to do so after the `sudo apt install` step in the next section.
Note that the address *must* include the leading "http://".
```bash
git config --global http.proxy [address]:[port]
```

### *System Updates and Required Packages*

Update the APT package lists and ensure your system is up to date.
```bash
sudo apt update && sudo apt upgrade
```

(TODO: from here to next section)

Install the following packages via the terminal.
```bash
sudo apt install -y build-essential libboost-all-dev clang libclang-dev clang-format clang-tidy cmake cmake-format doxygen git libgl1-mesa-dev qt6-base-dev libudev-dev
```

Package notes:

- `build-essential`: programs and libraries necessary for basic software development.
- `libboost-all-dev`: collection of useful C++ libraries ([link](https://www.boost.org/)).
- (TODO: necessary?) `clang` & `libclang-dev`: C/C++ compiler ([link](https://clang.llvm.org/)).
- `clang-format`: clang-based C++ formatter ([link](https://clang.llvm.org/docs/ClangFormat.html)).
- `clang-tidy`: clang-based C++ linter ([link](https://clang.llvm.org/extra/clang-tidy/)).
- `cmake`: cross-platform C++ build tool ([link](https://cmake.org/)).
- `cmake-format`: CMake formatter ([link](https://github.com/cheshirekow/cmake_format)).
- `doxygen`: C++ documentation generator ([link](https://www.doxygen.nl/)).
- `git`: popular open-source version control system ([link](https://git-scm.com)).
- (TODO: necessary?) `libgl1-mesa-dev`: open-source graphics library, used by Qt ([link](https://www.mesa3d.org/)).
- (TODO: necessary?) `qt6-base-dev`: Qt development libraries ([link](https://packages.ubuntu.com/jammy/qt6-base-dev)).
- `libudev-dev`: C++ library for enumerating local devices([link](https://www.freedesktop.org/software/systemd/man/latest/libudev.html)).

### *Required Drivers*

#### **Hokuyo URG (LIDAR)**

The `URG_USB_Driver.inf` setup file can be found in `external/urg-cpp-1.2.7/driver/`.

To install, simply right-click the file in Windows and select "Install".
For any troubleshooting, see the [official Microsoft docs](https://learn.microsoft.com/en-us/windows-hardware/drivers/ifs/using-an-inf-file-to-install-a-file-system-filter-driver).

#### **Maxon EPOS**

The driver should be automatically installed when connecting an EPOS controller to your PC (via USB) for the first time.

If manual installation is required (e.g., Device Manager shows "Unknown device" with a warning icon), follow the instructions in [the EPOS USB Driver Installation PDF](external/epos-6.8.1.0/driver/EPOS%20USB%20Driver%20Installation.pdf). All driver installation files are located in `external/epos-6.8.1.0/driver/`.

## Project Software

### *Qt Creator*

Apply for a [Qt educational license](https://www.qt.io/qt-educational-license#application) (make sure to select "Qt Edu for Developers").

Go to your [Account Page](https://account.qt.io/s/) -> Downloads and download the "Unified Qt Installer X.X.X. for Linux".
To run it, you must first give the `.run` file execution permissions (remember to replace the text in brackets).
```bash
chmod +x qt-unified-linux-x64-[ver]-online.run
./qt-unified-linux-x64-[ver]-online.run
```

> **_NOTE:_** If you're behind a proxy, open the settings menu (bottom left) and select "Manual proxy configuration".
Enter your proxy settings **without** the preceding `http://` (e.g., HTTP proxy: `proxy.noc.titech.ac.jp` Port: `3128`).

Login and follow the installation procedure, being mindful of the following:
- At the "Installation Folder" step, select "Qt Design Studio" and "Qt 6.x for desktop development".

#### *Add Qt Creator to PATH*

To run the GUI application from the terminal (via the command `qtcreator`), add the following to the end of your `~/.bashrc`.
```bash
if [ -d "$HOME/Qt/Tools" ]; then
    PATH="$PATH:$HOME/Qt/Tools/QtCreator/bin"
fi
```

#### **Troubleshooting**

##### *"No valid license available"*

Upon opening Qt Creator, you may get an error that there is no valid license available.
You can fix this via the Qt Maintenance Tool.

1. Open Qt Maintenance Tool as superuser.
```bash
sudo /opt/Qt/MaintenanceTool
```

2. Wait for the "Performing license check" message to give you the "Cancel" prompt, cancel it, then open the settings menu and select "Manual proxy configuration".
    - Since Qt Maintenance Tool is technically a different app, the proxy settings you entered in Qt Installer may not be set correctly.
3. Click "Retry", then login once the initial check finishes.
4. Select "Update components", then click "Next".
    - Once the operation finishes, you may get a message saying that there are no further updates required. This is fine -- your license info was still updated.
5. Close Qt Maintenance Tool.

You should now be able to use Qt Creator.

##### *"Nothing happens/I get errors when I try to run Qt Creator"*

If you are using a non-GNOME desktop environment (e.g., Windows's WSL2 or Mint's "Cinnamon" environment) you may run into problems with the Qt Creator UI.
This might be because you're missing some display-related packages that Qt expects.
Try the following catch-all install command for X11 display server protocol libraries.
```bash
sudo apt install '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev
```

If Qt Creator still doesn't run, try explicitly installing the following packages.
```bash
sudo apt install -y libfontconfig libxcb-glx0 libx11-xcb1 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-shape0 libxcb-xkb1 libxcb-xinerama0 libxkbcommon-x11-0 libegl1
```

> **_NOTE:_** Might be missing some, will have to check on laptop. (TODO)

See this [Stack Overflow thread](https://stackoverflow.com/questions/68036484/qt6-qt-qpa-plugin-could-not-load-the-qt-platform-plugin-xcb-in-even-thou) for more details.

### *Maxon EPOS Library (system-wide install)*

The necessary header file is already included in this project (see `external/epos-6.8.1.0/`).
However, for ease of compilation (specifically, using the `-lEposCmd` flag), follow the instructions below to install the EPOS library files on your system.

1. Open a terminal in the `external/epos-6.8.1.0/` directory and extract the EPOS Library archive **into a non-project directory** of your choice.
Navigate to the extracted directory.
```bash
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

<br><hr><br>

# Optional Items

## Project Software

### *VS Code*

To install, simply [download](https://code.visualstudio.com/download) and run the `.deb`.
```bash
sudo dpkg -i [package_name]
```

#### **Troubleshooting**

If you're prompted to "unlock a keyring" (by entering your Linux password) every time you start up VS Code, follow these instructions.

1. Open your display manager config file. If you're not sure what that is, look for a file ending in "dm" in the `/etc/pam.d` directory (e.g., `sddm`, `lightdm`).
```bash
sudo nano /etc/pam.d/[file ending in dm]
```

2. Check if the following "keyring" lines exist. If they do, simply remove the preceding dashes (`-`). Otherwise, append them to the end of their respective sections as shown below.
```txt
@include common-auth
auth    optional        pam_gnome_keyring.so
...
@include common-session
session optional        pam_gnome_keyring.so auto_start
```
