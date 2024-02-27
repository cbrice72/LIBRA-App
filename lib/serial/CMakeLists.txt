################################################################################
# @file   CMakeLists.txt
# @brief  Arduino serial USB module CMakeLists file.
#
# @author brice.c.aa
# @date   2024/2/27
################################################################################

# Create and configure the module
qt_add_library(
    serial
    STATIC
    # Header
    serial.h
    # Source
    serial.cpp
)

# Make sure the path to this directory is automatically added as an include
# path to all targets using this library
target_include_directories(
    serial
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)
