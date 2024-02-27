################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA HEBI actuators module CMakeLists file.
#
# @author brice.c.aa
# @date   2024/2/22
################################################################################

# Create and configure the module
qt_add_library(
    libra_hebi
    STATIC
    # Header
    libra_hebi.h
    # Source
    libra_hebi.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    libra_hebi
    # HEBI
    PRIVATE external/hebi-cpp-3.7.0/hebi-cpp/hebi/include
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    libra_hebi
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    # HEBI
    PRIVATE hebi hebic++
)
