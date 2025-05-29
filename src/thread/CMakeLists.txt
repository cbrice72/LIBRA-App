################################################################################
# @file   CMakeLists.txt
# @brief  Project threads CMakeLists file.
#
# @author brice.c.aa
################################################################################

# ========================= ACTUATOR THREAD =========================

# Check whether the third-party directories are defined
if(NOT DEFINED EPOS_DIR)
    message(
        FATAL_ERROR
            "EPOS_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable"
    )
endif()

if(NOT DEFINED HEBI_DIR)
    message(
        FATAL_ERROR
            "HEBI_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable"
    )
endif()

# Create and configure the module
qt_add_library(
    actuator_thread
    STATIC
    # Actuator Definitions
    ${CMAKE_CURRENT_SOURCE_DIR}/actuator_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/abstract_actuator_thread.h
    # EPOS (Maxon)
    ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.cpp
    # HEBI
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/src" "${HEBI_DIR}/hebi/include"
                      "${HEBI_DIR}/Eigen"
)

target_include_directories(
    actuator_thread
    # EPOS (Maxon)
    PUBLIC ${EPOS_INCLUDE_DIRS}
    # HEBI
    PUBLIC ${HEBI_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

if(BUILD_WITH_ROS2)
    # Add standard and LIBRA-specific ROS2 dependencies
    ament_target_dependencies(
        actuator_thread PUBLIC rclcpp
    )
    add_dependencies(actuator_thread libra_interfaces)

    # Ensure this module has access to the ROS2 message generator's include dir
    target_include_directories(actuator_thread PUBLIC ${LIBRA_INTERFACES_INCLUDE_DIR})

    # Link against LIBRA-specific interface(s)
    target_link_libraries(actuator_thread PUBLIC ${LIBRA_INTERFACES_CPP_TARGET})
endif()

# Tell the linker which libraries to link against
target_link_libraries(
    actuator_thread
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
    # EPOS (Maxon)
    PRIVATE -lEposCmd
    # HEBI
    PRIVATE hebi hebic++ m pthread
)

# ========================= LOG THREAD =========================

# Create and configure the module
qt_add_library(
    log_thread
    STATIC
    # Logging
    ${CMAKE_CURRENT_SOURCE_DIR}/log_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/log_thread.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries, if any
target_include_directories(
    log_thread
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    log_thread
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
)
