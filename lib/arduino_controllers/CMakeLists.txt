################################################################################
# @file   CMakeLists.txt
# @brief  Arduino manager module (QSerialPort wrapper) CMakeLists file.
#
# @author Christian Brice
################################################################################

# Declare module dependencies
set(QT_PACKAGES Core SerialPort)
find_package(QT NAMES Qt6 REQUIRED COMPONENTS ${QT_PACKAGES})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_PACKAGES})

# Create and configure the module
set(ARDUINO_DEVICES_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/generic_serial_device.h
    ${CMAKE_CURRENT_SOURCE_DIR}/generic_serial_device.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/flow_controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/flow_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/water_controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/water_controller.cpp
)

if(LIBRA_VERSION EQUAL 1)
    # UNUSED: old tip-mounted 2-axis manipulator (LIBRA-I)
    list(APPEND ARDUINO_DEVICES_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/manip_controller.h
        ${CMAKE_CURRENT_SOURCE_DIR}/manip_controller.cpp
    )
endif()

qt_add_library(arduino_manager STATIC ${ARDUINO_DEVICES_SOURCES})

target_include_directories(
    arduino_manager
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    arduino_manager
    # Project
    PUBLIC common logger
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    PRIVATE Qt${QT_VERSION_MAJOR}::SerialPort
)
