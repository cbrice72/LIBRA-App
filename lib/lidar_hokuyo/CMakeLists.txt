################################################################################
# @file   CMakeLists.txt
# @brief  Hokuyo (URG) LIDAR module CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Specify the Hokuyo URG library directory
set(URG_DIR ${CMAKE_SOURCE_DIR}/thirdparty/urg-1.2.7)

# Create and configure the module
qt_add_library(
    lidar_hokuyo
    STATIC
    # Header
    lidar_hokuyo.h
    # Source
    lidar_hokuyo.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
set(URG_INCLUDE_DIRS "${URG_DIR}/include/c" "${URG_DIR}/include/cpp")

target_include_directories(
    lidar_hokuyo
    # Hokuyo URG
    PUBLIC ${URG_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Import the Hokuyo URG library packaged with this project
set(URG_LIB ${URG_DIR}/lib/liburg_cpp.so)

add_library(urg_cpp SHARED IMPORTED GLOBAL)
set_target_properties(urg_cpp PROPERTIES IMPORTED_LOCATION ${URG_LIB})

# Tell the linker which libraries to link against
target_link_libraries(
    lidar_hokuyo
    # Hokuyo URG
    PRIVATE urg_cpp # -lurg_cpp
)
