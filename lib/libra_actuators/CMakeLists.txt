################################################################################
# @file   CMakeLists.txt
# @brief  Arm control thread module CMakeLists file.
#         Of the two libraries: `libra_actuators` defines types of actuators as
#         derivatives of a common template, and `arm_thread` implements them as
#         an interactive QThread object.
#
# @author brice.c.aa
################################################################################

# Only set up the example project if this CMakeLists.txt was directly invoked
# (i.e., this CMakeLists.txt was not called as a result of `add_subdirectory()`)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.16)

    project(
        LIBRA-Actuators-Example
        VERSION 0.1
        LANGUAGES CXX
    )

    # Project options
    set(EPOS_DIR "${CMAKE_SOURCE_DIR}/thirdparty/epos-6.8.1.0"
        CACHE PATH "Path to the latest EPOS API" FORCE)
    set(HEBI_DIR "${CMAKE_SOURCE_DIR}/thirdparty/hebi-cpp-3.11.1"
        CACHE PATH "Path to the latest HEBI API" FORCE)

    # CMake options
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # used by clang-tidy

    # Tell CMake to automatically handle Qt code generators and preprocessors
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)

    # Specify C++ standard to define available features
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    # Dependencies (installed on system)
    find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
endif()

# ---
# LIBRA Actuators
# ---

# EPOS Setup
# - Check whether the EPOS directory was defined
if(NOT DEFINED EPOS_DIR)
    message(FATAL_ERROR "EPOS_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable")
endif()
# - Include EPOS headers
set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")

# HEBI Setup
# - Check whether the HEBI directory was defined
if(NOT DEFINED HEBI_DIR)
    message(FATAL_ERROR "HEBI_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable")
endif()
# - Collect all HEBI source files (see comment in `qt_add_library` call below)
file(GLOB HEBI_SOURCE_FILES "${HEBI_DIR}/src/*.cpp")
# - Include headers from all libraries used by HEBI (TODO: is this overkill?)
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/Eigen" "${HEBI_DIR}/hebi/include"
                      "${HEBI_DIR}/rapidyaml" "${HEBI_DIR}/src"
)

# Create and configure the module
qt_add_library(
    libra_actuators
    STATIC
    # Header
    abstract_actuator.h
    epos_actuator.h
    hebi_actuator.h
    # Source
    epos_actuator.cpp
    hebi_actuator.cpp
    # HEBI src/ folder needs to be passed in during compilation/linking process
    # (see HEBI C++ API docs: https://files.hebi.us/docs/cpp/cpp-3.11.1/)
    ${HEBI_SOURCE_FILES}
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    libra_actuators
    # EPOS (Maxon)
    PUBLIC ${EPOS_INCLUDE_DIRS}
    # HEBI
    PUBLIC ${HEBI_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    libra_actuators
    # EPOS (Maxon)
    PRIVATE -lEposCmd
    # HEBI
    PRIVATE pthread hebic++
)

# ---
# Arm Thread
# ---

# Create and configure the module
qt_add_library(
    arm_thread
    STATIC
    # Header
    arm_thread.h
    actuator_defs.h
    # Source
    arm_thread.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    arm_thread
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    arm_thread
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
)

# ---
# Example
# ---

# Only build the example if this CMakeLists.txt was directly invoked
# (i.e., this CMakeLists.txt was not called as a result of `add_subdirectory()`)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # Add project subdirectories
    add_subdirectory(example-src)  # PROJECT_SOURCE

    # Add target executable
    if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
        qt_add_executable(LIBRA-Actuators-Example MANUAL_FINALIZATION ${PROJECT_SOURCE})
        # Define target properties for Android with Qt 6 as:
        #    set_property(TARGET LIBRA-Actuators-Example APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
        #                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
        # For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
    else()
        if(ANDROID)
            add_library(LIBRA-Actuators-Example SHARED ${PROJECT_SOURCE})
            # Define properties for Android with Qt 5 after find_package() calls as:
            #    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
        else()
            add_executable(LIBRA-Actuators-Example ${PROJECT_SOURCE})
        endif()
    endif()

    # Add all libraries directly used by this project to the include path
    target_include_directories(
        LIBRA-Actuators-Example
        # Project
        PRIVATE ${CMAKE_SOURCE_DIR}
        PRIVATE ${CMAKE_SOURCE_DIR}/src
    )

    # Tell the linker which libraries to link against
    target_link_libraries(
        LIBRA-Actuators-Example
        # Qt
        PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
        # Project
        arm_thread)

    # Specify platform-specific properties
    # ~~~
    if(${QT_VERSION} VERSION_LESS 6.1.0)
        # Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
        # If you are developing for iOS or macOS you should consider setting an
        # explicit, fixed bundle identifier manually though.
        set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.LIBRA-Actuators-Example)
    endif()
    set_target_properties(LIBRA-Actuators-Example PROPERTIES
        ${BUNDLE_ID_OPTION}
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
    )
    # ~~~

    # Auto-included by Qt
    include(GNUInstallDirs)

    # Specify installation settings
    install(TARGETS LIBRA-Actuators-Example
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    # Generate proper deployment settings for the target
    if(QT_VERSION_MAJOR EQUAL 6)
        qt_finalize_executable(LIBRA-Actuators-Example)
    endif()
endif()
