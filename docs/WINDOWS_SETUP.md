# Windows Setup

This document gives instructions on setting up a Linux development environment on a non-Linux machine (e.g., Windows).

- [Overview](#overview)
- [Virtual Machine (VM)](#virtual-machine-vm)
- [Windows Subsystem for Linux (WSL)](#windows-subsystem-for-linux-wsl)
    - [*Installation*](#installation)
    - [*Networking Mode*](#networking-mode)

## Overview

In order to run Ubuntu on Windows, you can choose between a Virtual Machine (VM) or the Windows Subsystem for Linux (WSL). There are a few differences you should be aware of.

| | Ability | Environment | USB Support | Shared Folder Support |
|---|---|---|---|---|
| **Native<br>Linux** | Full-featured | Runs natively on your PC | No special actions required | N/A |
| **VM** | Full-featured | Runs in separate environment | Select PC or VM on plug-in | Non-native; enable in VMWare settings + install [open-vm-tools](https://kb.vmware.com/s/article/2073803) |
| **WSL** | Lightweight | Runs natively in Windows | Non-native; install [USBIPD](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) + use every time |  Windows `C:\` drive located at `/mnt/c` |

## Virtual Machine (VM)

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

## Windows Subsystem for Linux (WSL)

### *Installation*

1. Open a PowerShell terminal and install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install).

    ```bash
    wsl --install -d Ubuntu-22.04
    ```

> ***NOTE:*** If you are getting the error "System Integrity policy has been violated", you will need to disable Smart App Control in Windows settings.

1. Once the installation is finished, enter the username and password you want to use when logging into the Ubuntu shell.

### *Networking Mode*

It is recommended that you change your WSL networking mode from the default `NAT` to `mirrored`, as this allows WSL to seamlessly integrate Windows network adapters (e.g., as required to use HEBI actuators via IP addressing).

1. Ensure WSL isn't running.

    ```ps
    # In a PowerShell window
    wsl.exe --shutdown
    ```

2. In your user folder (`C:\Users\<Username>`), create or edit the `.wslconfig` file. Add the following.

    ```txt
    [wsl2]
    networkingMode=mirrored
    ```

3. Configure HyperV (the WSL virtual machine manager) so that it allows all inbound connections (see the [Microsoft docs](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking) for more information).

    ```ps
    Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
    ```

4. Restart WSL.

    ```ps
    # In a PowerShell window
    wsl.exe
    ```

See the relevant [Microsoft docs page](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking) for more information.
