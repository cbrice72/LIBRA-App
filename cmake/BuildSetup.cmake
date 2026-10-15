################################################################################
# @file   BuildSetup.cmake
# @brief  Sets up the build tree for the requested LIBRA configuration.
#
# @author Christian Brice
################################################################################

# ========================= VALIDATION =========================

if(BUILD_WITH_ROS2)
    set(LIBRA_BUILD_TYPE "ros")
else()
    set(LIBRA_BUILD_TYPE "standalone")
endif()
set(LIBRA_VARIANT "libra-${LIBRA_VERSION}-${LIBRA_BUILD_TYPE}")

# Give each configuration its own build directory
if(DEFINED CACHE{LIBRA_CONFIGURED_VARIANT} AND NOT LIBRA_CONFIGURED_VARIANT
                                               STREQUAL LIBRA_VARIANT
)
    message(
        FATAL_ERROR
            "This build directory (${CMAKE_BINARY_DIR}) was built for "
            "${LIBRA_CONFIGURED_VARIANT}, not ${LIBRA_VARIANT}. Use a separate "
            "build directory for each configuration (the presets in "
            "CMakePresets.json use build/<configuration>/), or delete this one."
    )
endif()

set(LIBRA_CONFIGURED_VARIANT
    "${LIBRA_VARIANT}"
    CACHE INTERNAL "Configuration this build directory was created for"
)

# ========================= CMAKE SETUP =========================

# Set global target output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
