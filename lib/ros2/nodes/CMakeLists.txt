################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA ROS2 nodes CMakeLists file.
#
# @author brice.c.aa
################################################################################

cmake_minimum_required(VERSION 3.16)

project(
    libra_nodes
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

# ========================= MINIMAL PUB/SUB NODE =========================

# Create and configure the module
add_executable(
    minimal_pub_sub
    # Source
    ${CMAKE_CURRENT_SOURCE_DIR}/src/minimal_pub_sub.cpp
)

set_target_properties(minimal_pub_sub PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    minimal_pub_sub
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(minimal_pub_sub PUBLIC rclcpp std_msgs)

# Specify installation settings
install(TARGETS minimal_pub_sub)

# ========================= PACKAGE END =========================

ament_package()
