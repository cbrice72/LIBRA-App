################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA Hokuyo LIDAR module CMakeLists file.
#
# @author brice.c.aa
# @date   2024/3/6
################################################################################

option(USE_URG3D "Use 3D library (URG3D) instead of 2D library (URG)" OFF)

# Specify the Hokuyo URG library directory
if(USE_URG3D)
    set(URG_DIR ${CMAKE_SOURCE_DIR}/external/urg3d-1.1.1)
else()
    set(URG_DIR ${CMAKE_SOURCE_DIR}/external/urg-1.2.7)
endif()

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
if(USE_URG3D)
    set(URG_INCLUDE_DIRS ${URG_DIR}/include)
else()
    set(URG_INCLUDE_DIRS "${URG_DIR}/include/c ${URG_DIR}/include/cpp")
endif()

target_include_directories(
    libra_lidar
    # Hokuyo URG
    PUBLIC ${URG_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Import the Hokuyo URG library packaged with this project
if(USE_URG3D)
    set(URG_LIB ${URG_DIR}/lib/liburg3d.so)
else()
    set(URG_LIB ${URG_DIR}/lib/liburg_cpp.so)
endif()

add_library(urg_cpp SHARED IMPORTED GLOBAL)
set_target_properties(urg_cpp PROPERTIES IMPORTED_LOCATION ${URG_LIB})

# Tell the linker which libraries to link against
target_link_libraries(
    libra_lidar
    # Hokuyo URG
    PRIVATE urg_cpp #-lurg_cpp
)
