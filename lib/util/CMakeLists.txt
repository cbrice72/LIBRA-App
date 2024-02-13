################################################################################
# @file   CMakeLists.txt
# @brief  Utility module CMakeLists file.
#
# @author brice.c.aa
# @date   2023/12/15
################################################################################

# Create and configure the module
qt_add_library(
    util
    STATIC
    # Header
    utility.h
    # Source
    utility.cpp
)

target_include_directories(
    util
    # Ensure this module has access to the necessary 3rd-party libraries
    PRIVATE ${CMAKE_SOURCE_DIR}/external
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    util
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    # Maxon
    PRIVATE -lEposCmd
)
