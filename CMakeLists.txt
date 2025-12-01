################################################################################
# @file   CMakeLists.txt
# @brief  Root CMakeLists file.
#
# @author brice.c.aa
################################################################################

cmake_minimum_required(VERSION 3.16)
message(STATUS "Using CMake version: ${CMAKE_VERSION}")

project(
    libra_app
    VERSION 0.2
    LANGUAGES CXX C
)

# ========================= SETUP =========================

# General CMake config
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # used by clang-tidy
set(BUILD_TESTING OFF)

if(POLICY CMP0148) # ROS2 CMakeLists generate warnings about this
    cmake_policy(SET CMP0148 OLD)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(Options) # must come first as this defines build options

include(QtSetup)
if(BUILD_WITH_ROS2)
    include(Ros2Setup)
endif()
if(LIBRA_VERSION EQUAL 2)
    include(EposSetup)
endif()
include(HebiSetup)

# ========================= BUILD =========================

# Add project subdirectories (order matters!)
add_subdirectory(lib) # utilites, managers
add_subdirectory(src) # libra_app_gui, threads, dialogs

# ========================= INSTALL =========================

# Specify installation settings
include(GNUInstallDirs) # auto-included by Qt

install(
    TARGETS libra_app_gui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Copy runtime config files to the executable directory
add_custom_command(
    TARGET libra_app_gui
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBRA_SHARED}
            ${CMAKE_INSTALL_BINDIR}/shared
)

# Generate proper deployment settings for the target
qt_finalize_executable(libra_app_gui)

if(BUILD_WITH_ROS2)
    # Register the package with ament
    ament_package()
endif()

message(STATUS "LIBRA App executable installed in: ${CMAKE_INSTALL_BINDIR}")
