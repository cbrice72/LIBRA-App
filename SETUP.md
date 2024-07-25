# Setup

This document lists the necessary steps to set up a Visual Studio 2022 development environment for the LIBRA App on a Windows 11 machine.
Please contact Christian Brice ([email](mailto:brice.c.aa@m.titech.ac.jp)) with any questions or revision suggestions.

## Table of Contents

1. [Installing Visual Studio](#setting-up-visual-studio)
2. [Preparing Your Development Environment](#preparing-your-development-environment)
3. [Optional Items](#optional-items)

## Installing Visual Studio

1. Download [Visual Studio Community edition](https://visualstudio.microsoft.com/downloads/) and launch the installer.
2. Once you get to the component selection screen, do the following:
    - In "Workloads" -> "Desktop & Mobile", select **"Desktop development with C++"**
    - In "Individual components", select **"Python language support"**, **"MSBuild support for LLVM (clang-cl) toolset"**, **"C++ Clang Compiler for Windows"**, and **"Git for Windows"**
3. In the details tab on the right, expand the "Desktop development with C++" category and do the following:
    - Deselect "Test Adapter for Boost.Test", "Test Adapter for Google Test", and "Live Share"
    - Select "C++ MFC for latest v143 build tools"
4. Click "Install"; this may take a while.

## Preparing Your Development Environment

### *...*

TODO

### *...*

The following instructions were modified from these sources:
- [https://www.youtube.com/watch?v=t_YnACEPmrM](https://www.youtube.com/watch?v=t_YnACEPmrM)
- [https://github.com/PINTO0309/wsl2_linux_kernel_usbcam_enable_conf/](https://github.com/PINTO0309/wsl2_linux_kernel_usbcam_enable_conf/)

In a WSL2 shell:

1. Get your WSL2 kernel version. The set of numbers with periods in front of "microsoft" are your `TAGVERNUM`.
    ```bash
    uname -r -v
    ```
2. Prepare to modify the WSL2 kernel; install the following packages.
    ```bash
    sudo apt update && sudo apt upgrade -y
    sudo apt install -y \
      build-essential flex bison \
      libgtk-3-dev libelf-dev libncurses-dev autoconf \
      libudev-dev libtool zip unzip v4l-utils libssl-dev \
      python3-pip cmake git iputils-ping net-tools dwarves \
      guvcview python-is-python3 bc
    ```
3. Clone the source code of the WSL2 kernel you're running. Make sure you replace the text within the two sets of angled brackets (`<>`) with your `TAGVERNUM` and ***Windows*** username, respectively.
    ```bash
    cd /usr/src
    TAGVERNUM=<tagvernum> \
      && TAGVER=linux-msft-wsl-${TAGVERNUM} \
      && WINUSERNAME=<windows_username>
    sudo git clone --depth 1 -b ${TAGVER} \
      https://github.com/microsoft/WSL2-Linux-Kernel.git \
      ${TAGVERNUM}-microsoft-standard \
      && cd ${TAGVERNUM}-microsoft-standard
    ```
4. Extract the kernel configuration file and enter the Kernel-Mode Configuration Manager.
    ```bash
    sudo cp /proc/config.gz config.gz
    sudo gunzip config.gz
    sudo mv config .config
    sudo make menuconfig
    ```
5. Follow the steps below to add the necessary modules. Actions are selected using `Left`/`Right`, submenus are entered using `Enter`, and changes are made using `Space` (note: "enabling" an option usually requires pressing `Space` twice, until it shows an asterisk ("`*`")).
    - Enter "Device Drivers".
    - Scroll down to `Multimedia support`. Press `Space` **twice** to enable it ("`*`"), then enter the submenu.
    - Enable `Filter media drivers` (`Autoselect ancillary drivers` should automatically enable too).
    - Enter "Media device types", enable `Cameras and video grabbers`, then go back.
    - Enter "Video4Linux options", enable `V4L2 sub-device userspace API`, then go back.
    - Enter "Media drivers", enable `Media USB Adapters`, then enter the submenu.
    - Enable `USB Video Class (UVC)` (`UVC input events device support` should automatically appear and enable too) and `GSPCA based webcams`.
    - Exit all the way out, and select "Yes" to save your changes.
6. Build and install the modified kernel (this may take a while).
    ```bash
    sudo make -j$(nproc) KCONFIG_CONFIG=.config \
      && sudo make modules_install -j$(nproc) \
      && sudo make install -j$(nproc)
    ```
7. Copy the modified kernel to your Windows user folder (ensuring that there isn't one there already).
    ```bash
    sudo rm /mnt/c/Users/${WINUSERNAME}/vmlinux
    sudo cp /usr/src/${TAGVERNUM}-microsoft-standard/vmlinux /mnt/c/Users/${WINUSERNAME}/
    ```
8. Append the setting that will point Windows to the new kernel to your WSL2 config file (`.wslconfig`).
    ```bash
    cat << 'EOT' > /mnt/c/Users/${WINUSERNAME}/.wslconfig
    [wsl2]
    kernel=C:\\Users\\<windows username>\\vmlinux
    EOT
    ```

In a Windows Powershell Terminal (Admin):

1. Shut down WSL2.
    ```shell
    wsl --shutdown
    ```
2. Open a WSL2 shell once again.
    ```shell
    wsl
    ```
4. Check the kernel version.
    ```bash
    uname -r -v
    ```

If the built kernel has been loaded successfully, you will see **`+`** at the end of the kernel name.

## Optional Items

TODO
