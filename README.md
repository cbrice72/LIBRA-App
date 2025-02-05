# LIBRA Control App

The LIBRA robotic arm project is being developed for the internal survey of dangerous spaces.
This control application targets the first-generation LIBRA prototype: *LIBRA-I*.

Project link: https://github.com/christian-brice/LIBRA-App

## Table of Contents

- [Requirements](#requirements)
- [Optional](#optional)
    - [*VS Code Extensions*](#vs-code-extensions)
- [Usage](#usage)
    - [*LIBRA-I Control App*](#libra-i-control-app)
    - [*LIBRA-II Control App*](#libra-ii-control-app)
    - [*Code Formatting*](#code-formatting)
    - [*Code Linting*](#code-linting)
- [Documentation](#documentation)
- [About Us](#about-us)
- [Acknowledgements](#acknowledgements)
- [Contributing](#contributing)

## Requirements

| | Minimum | Recommended |
|---|---|---|
| **Operating System** | Ubuntu 22.04.4<br>(Jammy) | Ubuntu 24.04<br>(Noble) |
| **C++ Standard** | C++17 | C++17 |
| **Qt** | 6.1.0 | 6.7.2 |

## Optional

### *VS Code Extensions*

#### **Syntax Highlighting, Intellisense, Debugging**

- [C/C++](vscode:extension/ms-vscode.cpptools) by Microsoft
- [Better C++ Syntax](vscode:extension/jeff-hykin.better-cpp-syntax) by Jeff Hykin
- [CMake](vscode:extension/twxs.cmake) by twxs
- [CMake Tools](vscode:extension/ms-vscode.cmake-tools) by Microsoft
- [Error Lens](vscode:extension/usernamehw.errorlens) by Alexander

#### **Formatting and Linting**

- [Clang-Format](vscode:extension/xaver.clang-format) by Xaver Hellauer
- [cmake-format](vscode:extension/cheshirekow.cmake-format) by cheshirekow

#### **Convenience**

- [Doxygen Documentation Generator](vscode:extension/cschlosser.doxdocgen) by Christoph Schlosser
- [Doxygen Runner](vscode:extension/betwo.vscode-doxygen-runner) by betwo
- [Markdown All in One](vscode:extension/yzhang.markdown-all-in-one) by Yu Zhang

## Usage

### *LIBRA-I Control App*

Simply open `LIBRA_App.sln` in Visual Studio, build the solution, and run.

For detailed operation instructions, see [USAGE.md](/USAGE.md) in this directory.

### *LIBRA-II Control App*

TODO

### *Code Formatting*

This project uses [Clang-Format](https://clang.llvm.org/docs/ClangFormat.html) to format header (`.h`) and source (`.cpp`) files based on the parameters in the `.clang-format` file in this directory, and [cmake-format](https://github.com/cheshirekow/cmake_format) to format CMake (`CMakeLists.txt`) files based on the parameters in the `cmake-format.yaml` file in this directory.

If you installed the optional VS Code extensions of the same names, formatting will be automatically applied whenever you save a file.

### *Code Linting*

This project uses [Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/) to diagnose typical programming errors and enforce modern coding standards in source files based on the parameters in the `.clang-tidy` file in this directory.

If you installed the optional VS Code extension "C/C++", linting will automaticaly begin whenever you save a file, and suggestions will be shown in highlighted lines after processing finishes (may take a short while depending on the size of the file).

## Documentation

This project uses [Doxygen](https://www.doxygen.nl/) to generate documentation from source code using comment tags such as `@brief` and `@param`.

If you installed the optional VS Code extension "Doxygen Runner", simply open the command palette (`CtrlShift+P`) and select "Generate Doxygen documentation".
The HTML and LaTeX documentation will be output to the `docs` directory.

> **_NOTE:_** The HTML documentation can be viewed by opening `html/index.html`, however the files in the `latex` directory must first be compiled by a LaTeX compiler to generate a PDF.

## About Us

**Christian Brice** ([email](mailto:brice.c.aa@m.titech.ac.jp)) is a doctoral student in mechanical engineering at the Institute of Science Tokyo (formerly: Tokyo Institute of Technology).
The *LIBRA* project is the focus of his doctoral studies.

The **[Gen Endo Laboratory](www.robotics.mech.e.titech.ac.jp/gendo/en/)** is affiliated with the Department of Mechanical Engineering at the Institute of Science Tokyo.

## Acknowledgements

*LIBRA-I*, the first-generation LIBRA robot on which this project is based on, as well as the original Windows-based "LIBRA-I_App", were developed by **Yuto Goto** (a former member of the Gen Endo Laboratory).

## Contributing

1. Ensure Git is installed.
    ```bash
    sudo apt update
    sudo apt install git
    ```
    - If you're behind a proxy, you must also add your proxy settings to Git. In a terminal, input the following command (note: the `address` should be entered **with** the `http://` prefix).
        ```bash
        # (e.g., http://proxy.noc.titech.ac.jp:3128 for Gen Endo Lab)
        git config --global http.proxy <address>:<port>
        ```
2. In VS Code, open the command palette (`Ctrl+Shift+P`) and select "Git: Clone".
3. Enter `https://github.com/christian-brice/LIBRA-App.git` and select a directory to clone the project to.
