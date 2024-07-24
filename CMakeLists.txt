################################################################################
# @file   CMakeLists.txt
# @brief  Root CMakeLists file.
#
# @author brice.c.aa
# @date   2024/2/13
################################################################################

cmake_minimum_required(VERSION 3.5)

project(
    LIBRA-App-v2
    VERSION 0.1
    LANGUAGES CXX
)

# CMake options
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # used by clang-tidy

# Tell CMake to automatically handle Qt code generators and preprocessors
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Specify C++ standard to define available features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt packages
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)

# ROS packages
# ~~~
# TODO(brice.c.aa): add ROS2 capabilities?
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rmw_implementation REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# ~~~

# ROS messages
# ~~~
# TODO(brice.c.aa): add ROS2 capabilities?
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(epos_interfaces REQUIRED)
# ~~~

# Other packages
find_package(Boost REQUIRED COMPONENTS system thread)

# Add project subdirectories
add_subdirectory(lib)

# Specify project files
set(PROJECT_SOURCES
    # Header
    main_window.h
    serial_dialog.h
    # Source
    main.cpp
    main_window.cpp
    serial_dialog.cpp
    # Qt
    main_window.ui
    serial_dialog.ui
    resources.qrc
)

# Add target executable
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6) # Desktop, Qt6 (or higher)
    qt_add_executable(LIBRA-App-v2 MANUAL_FINALIZATION ${PROJECT_SOURCES})

    # ~~~
    # Define target properties for Android with Qt 6 as:
    #    set_property(TARGET LIBRA-App-v2 APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
    #                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
    # For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
    # ~~~

else() # Qt5 (or lower)
    if(ANDROID) # Mobile
        add_library(LIBRA-App-v2 SHARED ${PROJECT_SOURCES})

        # ~~~
        # Define properties for Android with Qt 5 after find_package() calls as:
        #    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
        # ~~~

    else() # Desktop
        add_executable(LIBRA-App-v2 ${PROJECT_SOURCES})
    endif()
endif()

# Copy HEBI actuator configurations to the executable directory
add_custom_command(
    TARGET LIBRA-App-v2
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/params
            ${CMAKE_INSTALL_BINDIR}/gains
)

# Add all 3rd-party libraries packaged with this project to the include path
target_include_directories(
    LIBRA-App-v2
    # Boost
    PRIVATE ${Boost_INCLUDE_DIRS}
)

# Tell the linker which libraries to link against
target_link_libraries(
    LIBRA-App-v2
    # Project
    # ~~~
    # TODO(brice.c.aa): add EPOS actuator
    # ~~~
    PRIVATE libra_hebi libra_lidar serial # TODO: util
    # ~~~
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Widgets
    # Other
    PRIVATE udev
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1. If
# you are developing for iOS or macOS you should consider setting an explicit,
# fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.LIBRA-App-v2)
endif()

# Specify platform-specific properties
# ~~~
set_target_properties(
    LIBRA-App-v2
    PROPERTIES ${BUNDLE_ID_OPTION}
               MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
               MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
               MACOSX_BUNDLE TRUE
               WIN32_EXECUTABLE TRUE
)
# ~~~

# TODO(brice.c.aa): auto-included by Qt -- is this necessary?
include(GNUInstallDirs)

# Specify installation settings
install(
    TARGETS LIBRA-App-v2
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Generate proper deployment settings for the target
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(LIBRA-App-v2)
endif()
