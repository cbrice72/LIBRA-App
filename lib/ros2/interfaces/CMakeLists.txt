################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA ROS2 interfaces (message types, services, etc.) CMakeLists file.
#
# @author brice.c.aa
################################################################################

cmake_minimum_required(VERSION 3.16)

project(
    libra_interfaces
    VERSION 0.1
    LANGUAGES CXX
)

# CMake options
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # used by clang-tidy

# Specify C++ standard to define available features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies (installed on system)
foreach(PKG ${ROS_CORE_PACKAGES})
    find_package(${PKG} REQUIRED)
endforeach()
foreach(PKG ${ROS_MSG_PACKAGES})
    find_package(${PKG} REQUIRED)
endforeach()

# ========================= TODO =========================

# ...

# ========================= PACKAGE END =========================

ament_package()
