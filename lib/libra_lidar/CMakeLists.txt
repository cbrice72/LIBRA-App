################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA Hokuyo LIDAR module CMakeLists file.
#
# @author brice.c.aa
# @date   2024/3/6
################################################################################

# Specify the Hokuyo URG library directory
set(URG_DIR ${CMAKE_SOURCE_DIR}/external/urg-cpp-1.2.7)

# Create and configure the module
qt_add_library(
    libra_lidar
    STATIC
    # Header
    libra_lidar.h
    # Source
    libra_lidar.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    libra_lidar
    # Hokuyo URG
    PUBLIC ${URG_DIR}/include/c
    PUBLIC ${URG_DIR}/include/cpp
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Import the Hokuyo URG library packaged with this project
add_library(urg_cpp SHARED IMPORTED GLOBAL)
set_target_properties(urg_cpp PROPERTIES IMPORTED_LOCATION ${URG_DIR}/lib/liburg_cpp.so)

# Tell the linker which libraries to link against
target_link_libraries(
    libra_lidar
    # Hokuyo URG
    PRIVATE urg_cpp #-lurg_cpp
)
