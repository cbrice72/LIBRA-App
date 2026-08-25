################################################################################
# @file   BuildSetup.cmake
# @brief  Determines the build tree for the requested LIBRA configuration.
#
# @author Christian Brice
################################################################################

# ========================= CMAKE SETUP =========================

if(BUILD_WITH_ROS2)
    set(LIBRA_BUILD_TYPE "ros")
else()
    set(LIBRA_BUILD_TYPE "standalone")
endif()

set(LIBRA_VARIANT_BUILD_DIR
    "${CMAKE_BINARY_DIR}/libra-${LIBRA_VERSION}-${LIBRA_BUILD_TYPE}"
    CACHE PATH "Directory for configuration-specific build artifacts" FORCE
)

# Ensure build subdirectory exists for this configuration
file(MAKE_DIRECTORY "${LIBRA_VARIANT_BUILD_DIR}")

# Set global target output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${LIBRA_VARIANT_BUILD_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRA_VARIANT_BUILD_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${LIBRA_VARIANT_BUILD_DIR}/lib")
