################################################################################
# @file   CMakeLists.txt
# @brief  Motion controller module CMakeLists file.
#
# @author Christian Brice
################################################################################

# Declare module dependencies
set(QT_PACKAGES Core)
find_package(QT NAMES Qt6 REQUIRED COMPONENTS ${QT_PACKAGES})
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${QT_PACKAGES})

# Create and configure the module
set(MOTION_CONTROLLER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/abstract_motion_controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/abstract_motion_controller.cpp
)

if(LIBRA_VERSION EQUAL 1)
    # TODO: not yet implemented
    # cmake-format: off
    #list(APPEND MOTION_CONTROLLER_SOURCES
    #    ${CMAKE_CURRENT_SOURCE_DIR}/libra1_motion_controller.h
    #    ${CMAKE_CURRENT_SOURCE_DIR}/libra1_motion_controller.cpp
    #)
    # cmake-format: on
elseif(LIBRA_VERSION EQUAL 2)
    list(APPEND MOTION_CONTROLLER_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/libra2_motion_controller.h
        ${CMAKE_CURRENT_SOURCE_DIR}/libra2_motion_controller.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libra2_calibration_file.h
        ${CMAKE_CURRENT_SOURCE_DIR}/libra2_calibration_file.cpp
    )
else()
    message(
        FATAL_ERROR
            "No motion controller available for LIBRA_VERSION: ${LIBRA_VERSION}"
    )
endif()

qt_add_library(motion_controller STATIC ${MOTION_CONTROLLER_SOURCES})

target_include_directories(
    motion_controller
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    motion_controller
    # Project
    PUBLIC common logger
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
)
