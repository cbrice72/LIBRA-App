################################################################################
# @file   CMakeLists.txt
# @brief  Arduino manager module (QSerialPort wrapper) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Declare module dependencies
set(QT_PACKAGES Core SerialPort)
find_package(QT NAMES Qt6 REQUIRED COMPONENTS ${QT_PACKAGES})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_PACKAGES})

# Create and configure the module
qt_add_library(
    arduino_manager
    STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_manager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_manager.cpp
)

target_include_directories(
    arduino_manager
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    arduino_manager
    # Project
    PUBLIC logger
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    PRIVATE Qt${QT_VERSION_MAJOR}::SerialPort
)
