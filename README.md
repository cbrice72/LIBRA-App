# LIBRA-II Control App

The LIBRA robotic arm project is being developed for the internal survey of dangerous spaces.
This control application targets the second-generation LIBRA robot, *LIBRA-II*.

Project link: https://github.com/christian-brice/LIBRA-II-App

1. [Requirements](#requirements)
2. [Usage](#usage)
3. [Documentation](#documentation)
4. [About Us](#about-us)
5. [Acknowledgements](#acknowledgements)
6. [Contributing](#contributing)

## Requirements

| | Minimum | Recommended |
|---|---|---|
| **Operating System** | Windows 10 | Windows 11 |
| **Visual Studio** | 2022 | 2022 |
| **MSVC Build Tools** | v143 | v143 |
| **C++ Standard** | C++14 | C++14 |

### *Optional*

#### **Packages**

- [LLVM (Clang Tools)](https://releases.llvm.org/download.html) 15.0.1 (or newer)

#### **Visual Studio Extensions**

- [Clang Power Tools](https://marketplace.visualstudio.com/items?itemName=caphyon.ClangPowerTools)
- [Doxygen Comments](https://marketplace.visualstudio.com/items?itemName=FinnGegenmantel.doxygenComments) by Finn Gegenmantel
- [Format document on Save](https://marketplace.visualstudio.com/items?itemName=mynkow.FormatdocumentonSave) by mynkow
- [VSDoxyHighlighter](https://marketplace.visualstudio.com/items?itemName=Sedenion.VSDoxyHighlighter) by Sedenion

## Usage

### *LIBRA-II Control App*

Simply open `LIBRA_App.sln` in Visual Studio, build the solution, and run.

For detailed operation instructions, see `USAGE.md` in this directory.

### *Code Formatting*

This project uses Visual Studio's built-in [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to format header and source files based on the parameters in the `.clang-format` file in this directory.

If you installed the optional extension "Format Document on Save", formatting will be automatically applied whenever you save a file.

### *Code Linting*

> **_NOTE:_** temporarily disabled; working off of previous `LIBRA-I_App` project, which did not follow code linting standards (as such there are many warnings and errors that slow development).

This project uses [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) to diagnose typical programming errors and enforce modern coding standards in source files based on the parameters in the `.clang-tidy` file in this directory.

If you installed the optional extension "Clang Power Tools", formatting can be applied on any source file by clicking the "Tidy" button in the toolbar, or via the shortcut Alt+Y.

> **_NOTE:_** clang-tidy will also print diagnostics for all `#include` files, including non-project (i.e., third-party) files. As a workaround, set your Error List filter to "Open Documents" to only show suggestions for relevant files.

## Documentation

This project uses [Doxygen](https://www.doxygen.nl/) to generate documentation from source code using comment tags such as `@brief` and `@param`.

Once you've installed Doxygen, you can either run it standalone and set this directory as the "Source Code Directory", or add it as an External Tool in Visual Studio (by following [these instructions](https://computingonplains.wordpress.com/doxygen-and-visual-studio/)).
The HTML and LaTeX documentation will be output to the `docs` directory.

> **_NOTE:_** The HTML documentation can be viewed by opening `html/index.html`, however the files in the `latex` directory must first be compiled by a LaTeX compiler to generate a PDF.

## About Us

**Christian Brice** ([email](mailto:brice.c.aa@m.titech.ac.jp)) is a doctoral student in mechanical engineering at the Tokyo Institute of Technology.
The *LIBRA-II* project is the focus of his doctoral studies.

The **[Gen Endo Laboratory](www.robotics.mech.e.titech.ac.jp/gendo/en/)** is affiliated with the Department of Mechanical Engineering at the Tokyo Institute of Technology.

## Acknowledgements

*LIBRA-I*, the first-generation LIBRA robot on which this project is based on, as well as the *LIBRA-I_App*, were developed by Yuto Goto (a former member of the Gen Endo Laboratory).

## Contributing

### *Preparation*

#### **Git for Windows**

First, ensure that "Git for Windows" is installed.
Open the Visual Studio Installer and click "Modify" under your installation of Visual Studio.
Then, in the "Individual Components" tab, make sure "Git for Windows" is checked.
To install it, simply click "Modify" at the bottom right.

#### **Giving proxy access to Git**

If you'd like to pull/push from within a proxy, you must also add your proxy settings to Git.
In Visual Studio, open a Developer PowerShell and input the following command.

```bash
git config --global http.proxy [ADDR]:[PORT]
```

... where `[ADDR]` is your proxy address and `[PORT]` is your proxy port (e.g., `http://proxy.noc.titech.ac.jp:3128` for Gen Endo Lab).

### *Accessing the Code*

Open the "Git Changes" window from the "View" tab at the top and click `Clone Repository...`.
Under "Repository location", copy/paste the following URL.
You can set "Path" to your liking.

```txt
https://github.com/christian-brice/LIBRA-II-App.git
```
