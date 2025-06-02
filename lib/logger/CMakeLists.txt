################################################################################
# @file   CMakeLists.txt
# @brief  Logger module CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Create and configure the module
set(SRC_FILES
    # Base (abstraction)
    ${CMAKE_CURRENT_SOURCE_DIR}/logger.h
    # Qt (default) Implementation
    ${CMAKE_CURRENT_SOURCE_DIR}/qt_logger.h
)

if(BUILD_WITH_ROS2)
    list(
        APPEND
        SRC_FILES
        # ROS2 Implementation
        ${CMAKE_CURRENT_SOURCE_DIR}/ros2_logger.h
    )
endif()

add_library(logger STATIC ${SRC_FILES})

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    logger
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    logger
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
)

if(BUILD_WITH_ROS2)
    # Additional includes and libraries for ROS2 integration
    ament_target_dependencies(logger PUBLIC rcutils)
endif()
