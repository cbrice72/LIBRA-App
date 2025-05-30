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
set(LIBRA_VERSION
    "TWO"
    CACHE
        STRING
        "Which prototype to build for: LIBRA-I (HEBI x5) or LIBRA-II (EPOS x1 + HEBI x1)"
)
set_property(CACHE LIBRA_VERSION PROPERTY STRINGS "ONE" "TWO")

option(BUILD_WITH_ROS2 "Build LIBRA App with ROS2 functionality" ON)

set(HEBI_DIR
    "${CMAKE_SOURCE_DIR}/thirdparty/hebi-cpp-3.11.1"
    CACHE PATH "Path to the HEBI API" FORCE
)

set(EPOS_DIR
    "${CMAKE_SOURCE_DIR}/thirdparty/epos-6.8.1.0"
    CACHE PATH "Path to the EPOS API" FORCE
)

# Validate project options and set internal vars (invisible to user)
if(LIBRA_VERSION STREQUAL "ONE")
    set(BUILD_LIBRA_ONE
        ON
        CACHE BOOL "" FORCE
    )
    set(BUILD_LIBRA_TWO
        OFF
        CACHE BOOL "" FORCE
    )
elseif(LIBRA_VERSION STREQUAL "TWO")
    set(BUILD_LIBRA_ONE
        OFF
        CACHE BOOL "" FORCE
    )
    set(BUILD_LIBRA_TWO
        ON
        CACHE BOOL "" FORCE
    )
else()
    message(
        FATAL_ERROR
            "Invalid LIBRA_VERSION: ${LIBRA_VERSION} - must be \"ONE\" or \"TWO\"!"
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

if(POLICY CMP0167) # don't use the older FindBoost module - include it directly
    cmake_policy(SET CMP0167 NEW)
endif()

# Tell CMake to automatically handle Qt code generators and preprocessors
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Specify C++ standard to define available features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies (installed on system)
find_package(Boost REQUIRED COMPONENTS system thread)

set(QT_PACKAGES Core Multimedia MultimediaWidgets SerialPort Widgets)
find_package(QT NAMES Qt6 REQUIRED COMPONENTS ${QT_PACKAGES})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_PACKAGES})

if(BUILD_WITH_ROS2)
    # Ensure the relevant preprocessor macros are triggered in C++ code
    add_compile_definitions(BUILD_WITH_ROS2)

    # Find ROS2 packages
    set(ROS_CORE_PACKAGES # used in other CMakeLists
        ament_cmake rclcpp rmw_implementation rosidl_default_generators
        CACHE INTERNAL "Required ROS2 core packages" FORCE
    )
    foreach(PKG ${ROS_CORE_PACKAGES})
        find_package(${PKG} REQUIRED)
    endforeach()

    set(ROS_MSG_PACKAGES # used in other CMakeLists
        builtin_interfaces std_msgs sensor_msgs
        CACHE INTERNAL "Required ROS2 message packages" FORCE
    )
    foreach(PKG ${ROS_MSG_PACKAGES})
        find_package(${PKG} REQUIRED)
    endforeach()
endif()

# Dependencies (packaged with this project)
add_subdirectory(${HEBI_DIR})
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/src" "${HEBI_DIR}/hebi/include"
                      "${HEBI_DIR}/Eigen"
)

if(BUILD_LIBRA_TWO)
    # NOTE: there is no `add_subdirectory()` call for EPOS since it doesn't
    # provide a CMakeLists.txt, only a binary to link against
    set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")
endif()

# ========================= BUILD =========================

# Add project subdirectories
add_subdirectory(lib) # camera_manager, util, [ROS2 interfaces & nodes]
add_subdirectory(src) # PROJECT_SOURCE, actuator_thread, log_thread

# Add target executable
qt_add_executable(libra_app MANUAL_FINALIZATION ${PROJECT_SOURCE})

# ~~~
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET libra_app APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
# ~~~

# Copy app and actuator configurations to the executable directory
if(BUILD_LIBRA_ONE)
    set(LIBRA_SHARED ${CMAKE_SOURCE_DIR}/shared/libra-1)
else() # BUILD_LIBRA_TWO
    set(LIBRA_SHARED ${CMAKE_SOURCE_DIR}/shared/libra-2)
endif()

add_custom_command(
    TARGET libra_app
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBRA_SHARED}
            ${CMAKE_INSTALL_BINDIR}/shared
)

# Add all libraries directly used by this project to the include path
target_include_directories(
    libra_app
    # Project
    PRIVATE ${CMAKE_SOURCE_DIR}/src
    # Boost
    PRIVATE ${Boost_INCLUDE_DIRS}
    # HEBI
    PRIVATE ${HEBI_INCLUDE_DIRS}
)

if(BUILD_LIBRA_TWO)
    target_include_directories(
        libra_app
        # EPOS (Maxon)
        PRIVATE ${EPOS_INCLUDE_DIRS}
    )
endif()

if(BUILD_WITH_ROS2)
    # Let ament take care of adding headers, libraries, and sub-dependencies
    ament_target_dependencies(
        libra_app PUBLIC ${ROS_CORE_PACKAGES} ${ROS_MSG_PACKAGES}
    )
endif()

# Tell the linker which libraries to link against
target_link_libraries(
    libra_app
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
    # HEBI
    PRIVATE hebi hebic++ m pthread
    # Linux
    PRIVATE udev # TODO(brice.c.aa): is this necessary?
)

if(BUILD_LIBRA_TWO)
    target_link_libraries(
        libra_app
        # EPOS (Maxon)
        PRIVATE -lEposCmd
    )
endif()

# ========================= INSTALL =========================

# Specify platform-specific properties
# ~~~
set_target_properties(
    libra_app
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
    TARGETS libra_app
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Generate proper deployment settings for the target
qt_finalize_executable(libra_app)

if(BUILD_WITH_ROS2)
    # Register the package with ament
    ament_package()
endif()
