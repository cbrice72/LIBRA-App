################################################################################
# @file   QtSetup.cmake
# @brief  CMake module for finding and configuring Qt libraries.
#
# @author Christian Brice
################################################################################

# ========================= VALIDATION =========================

if(NOT DEFINED QT_DIR)
    message(
        AUTHOR_WARNING
            "QT_DIR is not defined, meaning you may be attempting to run build this project outside of Qt Creator."
            "Doing so means the necessary Qt CMake modules are not automatically appended to \"CMAKE_PREFIX_PATH\"."
            "It is recommended to build this project through Qt Creator."
            "\nCMake will now attempt to locate a Qt6 installation...\n"
    )

    # Check for both user-level and system-wide installs
    set(QT_INSTALL_DIRS "$ENV{HOME}/Qt" "/opt/Qt")
    set(QT_DIR_CUSTOM "")
    set(QT_VERSION_CUSTOM "")

    foreach(install_dir IN LISTS QT_INSTALL_DIRS)
        if(EXISTS "${install_dir}")
            set(QT_DIR_CUSTOM "${install_dir}")

            # Look for versioned Qt6 directories (e.g., 6.9.0)
            file(
                GLOB qt_versions
                RELATIVE "${install_dir}" "${install_dir}/6.*"
            )
            list(
                SORT qt_versions
                COMPARE NATURAL
                ORDER DESCENDING
            )

            foreach(version IN LISTS qt_versions)
                if(EXISTS "${install_dir}/${version}/gcc_64/lib/cmake")
                    set(QT_VERSION_CUSTOM "${version}")
                    message(
                        STATUS
                            "Found Qt ${QT_VERSION_CUSTOM} at ${install_dir}"
                    )
                    break()
                endif()
            endforeach()

            # Only stop searching if both dir and version were found
            if(QT_DIR_CUSTOM AND QT_VERSION_CUSTOM)
                break()
            endif()
        endif()
    endforeach()

    # Add Qt CMake modules to CMake's environment
    if(QT_DIR_CUSTOM AND QT_VERSION_CUSTOM)
        list(APPEND CMAKE_PREFIX_PATH
             "${QT_DIR_CUSTOM}/${QT_VERSION_CUSTOM}/gcc_64/lib/cmake"
        )
    else()
        message(
            WARNING
                "Could not auto-detect Qt6. Please define QT_DIR manually."
        )
    endif()
endif()

# ========================= CMAKE SETUP =========================

# Tell CMake to automatically handle Qt code generators and preprocessors
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ========================= DEPENDENCIES =========================

set(QT_PACKAGES
    Core
    Gui
    Multimedia
    MultimediaWidgets
    SerialPort
    Widgets
)

find_package(QT NAMES Qt6 REQUIRED COMPONENTS ${QT_PACKAGES})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_PACKAGES})

# Validate version
if(Qt6Core_VERSION VERSION_LESS "6.8.0")
    message(FATAL_ERROR
        "This project requires Qt 6.8.0+. Found: ${Qt6Core_VERSION}"
    )
endif()
