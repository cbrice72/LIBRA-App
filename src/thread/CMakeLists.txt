################################################################################
# @file   CMakeLists.txt
# @brief  Project threads CMakeLists file.
#
# @author brice.c.aa
################################################################################

# ========================= LOG THREAD =========================

# Create and configure the module
qt_add_library(
    log_thread
    STATIC
    # Logging
    ${CMAKE_CURRENT_SOURCE_DIR}/log_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/log_thread.cpp
)

# Ensure this module packages the necessary headers
target_include_directories(
    log_thread
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    log_thread
    # Project
    PUBLIC util
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
)

# ========================= ARDUINO THREAD =========================

# Create and configure the module
qt_add_library(
    arduino_thread
    STATIC
    # Logging
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_thread.cpp
)

# Ensure this module packages the necessary headers
target_include_directories(
    arduino_thread
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    arduino_thread
    # Project
    PUBLIC logger
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
    PRIVATE Qt${QT_VERSION_MAJOR}::SerialPort
)

# ========================= ACTUATOR THREAD =========================

# Check whether the third-party directories are defined
if(LIBRA_VERSION EQUAL 2 AND NOT DEFINED EPOS_DIR)
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
set(SRC_FILES
    # Actuator Definitions
    ${CMAKE_CURRENT_SOURCE_DIR}/actuator_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/abstract_actuator_thread.h
    # HEBI
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.h
    ${CMAKE_CURRENT_SOURCE_DIR}/hebi_thread.cpp
)

if(LIBRA_VERSION EQUAL 2)
    list(
        APPEND
        SRC_FILES
        # EPOS (Maxon)
        ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.h
        ${CMAKE_CURRENT_SOURCE_DIR}/epos_thread.cpp
    )
endif()

qt_add_library(actuator_thread STATIC ${SRC_FILES})

# Ensure this module packages the necessary headers
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/src" "${HEBI_DIR}/hebi/include"
                      "${HEBI_DIR}/Eigen"
)

target_include_directories(
    actuator_thread
    # HEBI
    PUBLIC ${HEBI_INCLUDE_DIRS}
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    actuator_thread
    # Project
    PUBLIC logger util
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
    # HEBI
    PRIVATE hebi hebic++ m pthread
)

if(LIBRA_VERSION EQUAL 2)
    # Additional includes and libraries for the EPOS actuator
    set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")

    target_include_directories(
        actuator_thread
        # EPOS (Maxon)
        PUBLIC ${EPOS_INCLUDE_DIRS}
    )

    target_link_libraries(
        actuator_thread
        # EPOS (Maxon)
        PRIVATE -lEposCmd
    )
endif()

if(BUILD_WITH_ROS2)
    # Additional includes and libraries for ROS2 integration
    target_include_directories(
        actuator_thread PUBLIC ${LIBRA_INTERFACES_INCLUDE_DIR}
    )

    target_link_libraries(actuator_thread PUBLIC ${LIBRA_INTERFACES_CPP_TARGET})
    ament_target_dependencies(actuator_thread PUBLIC rclcpp)
endif()
