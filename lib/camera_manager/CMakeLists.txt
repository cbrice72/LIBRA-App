################################################################################
# @file   CMakeLists.txt
# @brief  Camera manager module (QMediaCaptureSession wrapper) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Create and configure the module
qt_add_library(
    camera_manager
    STATIC
    # Header
    camera_manager.h
    # Source
    camera_manager.cpp
)

target_include_directories(
    camera_manager
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    camera_manager
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    PRIVATE Qt${QT_VERSION_MAJOR}::Multimedia
    PRIVATE Qt${QT_VERSION_MAJOR}::MultimediaWidgets
)
