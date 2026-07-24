################################################################################
# @file   CMakeLists.txt
# @brief  Root CMakeLists file.
#
# @author Christian Brice
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

include(GNUInstallDirs) # auto-included by Qt

# ========================= BUILD =========================

# Add project subdirectories (order matters!)
add_subdirectory(lib) # utilites, managers
add_subdirectory(src) # libra_app_gui

# ========================= FINALIZE =========================

if(BUILD_WITH_ROS2)
    # Register the package with ament
    ament_package()
endif()

message(STATUS "LIBRA App installed at: ${CMAKE_INSTALL_BINDIR}")
