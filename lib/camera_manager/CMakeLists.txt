################################################################################
# @file   CMakeLists.txt
# @brief  Camera manager module (QMediaCaptureSession wrapper) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Declare module dependencies
set(QT_PACKAGES Core Multimedia MultimediaWidgets)
find_package(QT NAMES Qt6 REQUIRED COMPONENTS ${QT_PACKAGES})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_PACKAGES})

if(BUILD_WITH_ROS2)
    # Additional dependencies for ROS2 integration
    
    # (1) ROS2 Core, Messages
    set(ROS_CORE_PACKAGES ament_cmake rclcpp)
    foreach(PKG ${ROS_CORE_PACKAGES})
        find_package(${PKG} REQUIRED)
    endforeach()

    set(ROS_MSG_PACKAGES sensor_msgs)
    foreach(PKG ${ROS_MSG_PACKAGES})
        find_package(${PKG} REQUIRED)
    endforeach()

    # (2) ROS2-adjacent libraries
    find_package(cv_bridge REQUIRED)
    find_package(OpenCV REQUIRED)
endif()

# Create and configure the module
qt_add_library(
    camera_manager
    STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_manager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_manager.cpp
)

target_include_directories(
    camera_manager
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    camera_manager
    # Project
    PUBLIC logger
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    PRIVATE Qt${QT_VERSION_MAJOR}::Multimedia
    PRIVATE Qt${QT_VERSION_MAJOR}::MultimediaWidgets
)

if(BUILD_WITH_ROS2)
    # Additional includes and libraries for ROS2 integration
    target_include_directories(camera_manager PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(camera_manager PRIVATE ${OpenCV_LIBS})
    ament_target_dependencies(
        camera_manager PUBLIC rclcpp sensor_msgs cv_bridge
    )
endif()
