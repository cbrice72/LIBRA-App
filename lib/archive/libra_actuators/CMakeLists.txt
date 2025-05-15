################################################################################
# @file   CMakeLists.txt
# @brief  Actuator control thread modules CMakeLists file.
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

    # Define the actual LIBRA project root, for convenience
    set(ROOT_DIR ${CMAKE_SOURCE_DIR}/../..)
    get_filename_component(ROOT_DIR "${ROOT_DIR}" REALPATH)

    # Project options
    set(EPOS_DIR "${ROOT_DIR}/thirdparty/epos-6.8.1.0"
        CACHE PATH "Path to the latest EPOS API" FORCE)
    set(HEBI_DIR "${ROOT_DIR}/thirdparty/hebi-cpp-3.11.1"
        CACHE PATH "Path to the latest HEBI API" FORCE)

    # ~~~
    # - NOTE: the following is necessary since HEBI_DIR is an out-of-tree source
    #         ("out-of-tree" as in HEBI_DIR is not a subdir of CMAKE_SOURCE_DIR)
    # ~~~
    set(hebi_cpp_build_dir hebi-cpp-${CMAKE_BUILD_TYPE})
    string(TOLOWER ${hebi_cpp_build_dir} hebi_cpp_build_dir)
    get_filename_component(hebi_cpp_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${hebi_cpp_build_dir}" REALPATH)

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

    # Dependencies (packaged with this project)
    add_subdirectory(${HEBI_DIR} ${hebi_cpp_build_dir})
endif()

# ---
# EPOS (Maxon) Thread
# ---

# Check whether the EPOS directory is defined
if(NOT DEFINED EPOS_DIR)
    message(FATAL_ERROR "EPOS_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable")
endif()

# Create and configure the module
qt_add_library(
    epos_thread
    STATIC
    # Header
    ${CMAKE_CURRENT_SOURCE_DIR}/abstract_actuator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/actuator_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.h
    # Source
    ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")
target_include_directories(
    epos_thread
    # EPOS (Maxon)
    PUBLIC ${EPOS_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    epos_thread
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
    # EPOS (Maxon)
    PRIVATE -lEposCmd
)

# ---
# HEBI Thread
# ---

# Check whether the HEBI directory is defined
if(NOT DEFINED HEBI_DIR)
    message(FATAL_ERROR "HEBI_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable")
endif()
# - Include headers from all libraries used by HEBI


# Create and configure the module
qt_add_library(
    hebi_thread
    STATIC
    # Header
    ${CMAKE_CURRENT_SOURCE_DIR}/abstract_actuator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/actuator_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.h
    # Source
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/src" "${HEBI_DIR}/hebi/include" "${HEBI_DIR}/Eigen")
target_include_directories(
    hebi_thread
    # HEBI
    PUBLIC ${HEBI_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    hebi_thread
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
    # HEBI
    PRIVATE hebi hebic++ m pthread
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

    # Copy app and actuator configurations to the executable directory
    add_custom_command(
        TARGET LIBRA-Actuators-Example
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${ROOT_DIR}/shared/libra-2
                ${CMAKE_INSTALL_BINDIR}/shared
    )

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
        PRIVATE epos_thread
        PRIVATE hebi_thread
    )

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
