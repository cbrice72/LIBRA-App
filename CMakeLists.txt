################################################################################
# @file   CMakeLists.txt
# @brief  Root CMakeLists file.
#
# @author brice.c.aa
################################################################################

cmake_minimum_required(VERSION 3.16)

project(
    libra_app
    VERSION 0.1
    LANGUAGES CXX C
)

# ========================= CONFIG =========================

# Project options
# - Configurable by user
set(LIBRA_VERSION
    2
    CACHE
        STRING
        "Which prototype to build for: 1 = LIBRA-I (HEBI x5), 2 = LIBRA-II (EPOS x1 + HEBI x1)"
)
set_property(CACHE LIBRA_VERSION PROPERTY STRINGS 1 2)

option(BUILD_WITH_ROS2 "Build LIBRA App with ROS2 functionality" ON)

# - Internal (shouldn't need to be changed by user)
set(HEBI_DIR
    "${CMAKE_SOURCE_DIR}/thirdparty/hebi-cpp-3.11.1"
    CACHE PATH "Path to the HEBI API" FORCE  # included in this project
)

set(EPOS_DIR
    "${CMAKE_SOURCE_DIR}/thirdparty/epos-6.8.1.0"
    CACHE PATH "Path to the EPOS API" FORCE  # included in this project
)

# Validate project options and ensure the relevant preprocessor directives are triggered in C++ code
if(LIBRA_VERSION EQUAL 1 OR LIBRA_VERSION EQUAL 2)
    add_compile_definitions(LIBRA_VERSION=${LIBRA_VERSION})
else()
    message(
        FATAL_ERROR
            "Invalid LIBRA_VERSION: ${LIBRA_VERSION} - must be 1 (for LIBRA-I) or 2 (for LIBRA-II)!"
    )
endif()

if(BUILD_WITH_ROS2)
    if(NOT DEFINED ENV{AMENT_PREFIX_PATH})
        message(
            FATAL_ERROR
                "It seems a ROS 2 environment has not been sourced (AMENT_PREFIX_PATH is not set). Please run 'source /opt/ros/<distro>/setup.bash' before opening QtCreator."
        )
    endif()

    if(NOT "$ENV{ROS_VERSION}" STREQUAL "2")
        message(
            FATAL_ERROR
                "Couldn't find a valid ROS 2 installation (ROS_VERSION is not 2)."
        )
    endif()

    add_compile_definitions(BUILD_WITH_ROS2)
endif()

# Check if CMake was called outside of a Qt environment (i.e., QtCreator)
if(NOT DEFINED QT_DIR)
    message(
        AUTHOR_WARNING
            "\nQT_DIR is not defined, meaning you may be attempting to run \"colcon build\" outside of QtCreator.\n"
            "Doing so means the necessary Qt CMake modules are not automatically appended to \"CMAKE_PREFIX_PATH\".\n"
            "It is recommended to build this project through QtCreator.\n\n"
            "CMake will now attempt to locate a Qt6 installation...\n"
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
                RELATIVE "${install_dir}"
                "${install_dir}/6.*"
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
                            "Found Qt ${QT_VERSION_CUSTOM} at ${QT_DIR_CUSTOM}"
                    )
                    break()
                endif()
            endforeach()

            # Just in case: DON'T break if we didn't also find a valid directory
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
                "Could not auto-detect a Qt6 installation. Please define QT_DIR manually."
        )
    endif()
endif()

# CMake options and policies
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # used by clang-tidy
set(BUILD_TESTING OFF)

if(POLICY CMP0148) # ROS2 CMakeLists generate warnings about this
    cmake_policy(SET CMP0148 OLD)
endif()

# Tell CMake to automatically handle Qt code generators and preprocessors
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Specify C++ standard to define available features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies (installed on system)
set(QT_PACKAGES Core Multimedia MultimediaWidgets SerialPort Widgets)
find_package(QT NAMES Qt6 REQUIRED COMPONENTS ${QT_PACKAGES})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_PACKAGES})

if(BUILD_WITH_ROS2)
    # Find ROS2 packages
    set(ROS_CORE_PACKAGES ament_cmake rclcpp rcutils rmw_implementation rosidl_default_generators)
    foreach(PKG ${ROS_CORE_PACKAGES})
        find_package(${PKG} REQUIRED)
    endforeach()

    set(ROS_MSG_PACKAGES builtin_interfaces std_msgs sensor_msgs)
    foreach(PKG ${ROS_MSG_PACKAGES})
        find_package(${PKG} REQUIRED)
    endforeach()
endif()

# Dependencies (packaged with this project)
add_subdirectory(${HEBI_DIR})
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/src" "${HEBI_DIR}/hebi/include"
                      "${HEBI_DIR}/Eigen"
)

if(LIBRA_VERSION EQUAL 2)
    # NOTE: there is no `add_subdirectory()` call for EPOS since it doesn't
    # provide a CMakeLists.txt, only a binary to link against
    set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")
endif()

# ========================= BUILD =========================

# Add project subdirectories
add_subdirectory(lib) # camera_manager, util, [ROS2 interfaces & nodes]
add_subdirectory(src) # PROJECT_SOURCE, actuator_thread, log_thread

# Add GUI target executable
qt_add_executable(libra_app_gui MANUAL_FINALIZATION ${PROJECT_SOURCE})

# ~~~
# Define GUI target properties for Android with Qt 6 as:
#    set_property(TARGET libra_app_gui APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
# ~~~

# Copy app and actuator configurations to the executable directory
if(LIBRA_VERSION EQUAL 1)
    set(LIBRA_SHARED ${CMAKE_SOURCE_DIR}/shared/libra-1)
else() # assume LIBRA_VERSION EQUAL 2
    set(LIBRA_SHARED ${CMAKE_SOURCE_DIR}/shared/libra-2)
endif()

add_custom_command(
    TARGET libra_app_gui
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBRA_SHARED}
            ${CMAKE_INSTALL_BINDIR}/shared
)

# Add all libraries directly used by the GUI to the include path
target_include_directories(libra_app_gui PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Tell the linker which libraries to link against
target_link_libraries(
    libra_app_gui
    # Project
    PRIVATE camera_manager util actuator_thread log_thread
    # Qt
    # ~~~
    # Compilation breaks if Multimedia and MultimediaWidgets aren't linked to
    # the main executable, even though they're only needed by camera_manager.
    # Why does this happen? I don't know. I'm tired.
    # ~~~
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    PRIVATE Qt${QT_VERSION_MAJOR}::Multimedia
    PRIVATE Qt${QT_VERSION_MAJOR}::MultimediaWidgets
    PRIVATE Qt${QT_VERSION_MAJOR}::SerialPort
    PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    # Linux
    PRIVATE udev # TODO(brice.c.aa): is this necessary?
)

# ========================= INSTALL =========================

# Specify platform-specific properties
# ~~~
set_target_properties(
    libra_app_gui
    PROPERTIES ${BUNDLE_ID_OPTION} MACOSX_BUNDLE_BUNDLE_VERSION
               ${PROJECT_VERSION} MACOSX_BUNDLE_SHORT_VERSION_STRING
               ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR} MACOSX_BUNDLE
               TRUE WIN32_EXECUTABLE
               TRUE
)
# ~~~

# Specify installation settings
include(GNUInstallDirs) # auto-included by Qt

install(
    TARGETS libra_app_gui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Generate proper deployment settings for the target
qt_finalize_executable(libra_app_gui)

if(BUILD_WITH_ROS2)
    # Register the package with ament
    ament_package()
endif()
