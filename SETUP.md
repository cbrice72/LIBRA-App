# Setup

This document lists the necessary steps to set up a WSL2 (Windows Subsystem for Linux v2, running Ubuntu) development environment for the LIBRA App on a Windows 11 machine.
Please contact Christian Brice ([email](mailto:brice.c.aa@m.titech.ac.jp)) with any questions or revision suggestions.

## Table of Contents

1. [Setting up WSL2](#setting-up-wsl2)
2. [Preparing Your Development Environment](#preparing-your-development-environment)
    - [Proxy Settings](#proxy-settings)
    - [System Updates and Required Packages](#system-updates-and-required-packages)
3. [Project Software](#project-software)
    - [Qt Creator](#qt-creator)
    - [Maxon EPOS Library (system-wide install)](#maxon-epos-library-system-wide-install)
4. [Optional Items](#optional-items)

## Setting Up WSL2

1. TODO

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
sudo apt install -y build-essential clang libclang-dev clang-format clang-tidy cmake cmake-format doxygen git libgl1-mesa-dev qt6-base-dev
```

Package notes:

- `build-essential`: programs and libraries necessary for basic software development.
- (TODO: necessary?) `clang` & `libclang-dev`: C/C++ compiler ([link](https://clang.llvm.org/)).
- `clang-format`: clang-based C++ formatter ([link](https://clang.llvm.org/docs/ClangFormat.html)).
- `clang-tidy`: clang-based C++ linter ([link](https://clang.llvm.org/extra/clang-tidy/)).
- `cmake`: cross-platform C++ build tool ([link](https://cmake.org/)).
- `cmake-format`: CMake formatter ([link](https://github.com/cheshirekow/cmake_format)).
- `doxygen`: C++ documentation generator ([link](https://www.doxygen.nl/)).
- `git`: popular open-source version control system ([link](https://git-scm.com)).
- (TODO: necessary?) `libgl1-mesa-dev`: open-source graphics library, used by Qt ([link](https://www.mesa3d.org/)).
- (TODO: necessary?) `qt6-base-dev`: Qt development libraries ([link](https://packages.ubuntu.com/jammy/qt6-base-dev)).

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
