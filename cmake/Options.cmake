################################################################################
# @file   Options.cmake
# @brief  CMake module defining project build options.
#
# @author brice.c.aa
################################################################################

# ========================= PUBLIC OPTIONS =========================

# The LIBRA robot this app will control (string option)
set(LIBRA_VERSION
    1
    CACHE STRING
    "Which prototype to build for: 1 = LIBRA-I (HEBI x5), 2 = LIBRA-II (EPOS x1 + HEBI x1)"
)
set_property(CACHE LIBRA_VERSION PROPERTY STRINGS 1 2)
message(STATUS "Selected LIBRA version: ${LIBRA_VERSION}")

# Whether to enable ROS2 functionality (boolean option)
option(BUILD_WITH_ROS2 "Build LIBRA App with ROS2 functionality" ON)
message(STATUS "ROS2 integration: ${BUILD_WITH_ROS2}")

# ========================= CMAKE SETUP & VALIDATION =========================

# Adding compile definitions ensures the relevant preprocessor directives are
# triggered in the C++ code
if(LIBRA_VERSION EQUAL 1 OR LIBRA_VERSION EQUAL 2)
    add_compile_definitions(LIBRA_VERSION=${LIBRA_VERSION})
else()
    message(
        FATAL_ERROR
            "Invalid LIBRA_VERSION: ${LIBRA_VERSION} - must be 1 (for LIBRA-I) or 2 (for LIBRA-II)!"
    )
endif()

if(BUILD_WITH_ROS2)
    add_compile_definitions(BUILD_WITH_ROS2)
    # NOTE: validation is done in Ros2Setup.cmake
endif()

# ========================= INTERNAL OPTIONS =========================
# These options are for internal use and should not need to be changed by users

set(HEBI_DIR
    "${CMAKE_SOURCE_DIR}/thirdparty/hebi-cpp-3.13.0"
    CACHE PATH "Path to the HEBI API" FORCE
)

set(EPOS_DIR
    "${CMAKE_SOURCE_DIR}/thirdparty/epos-6.8.1.0"
    CACHE PATH "Path to the EPOS API" FORCE
)

set(LIBRA_SHARED
    ${CMAKE_SOURCE_DIR}/shared/libra-${LIBRA_VERSION}
    CACHE PATH "Path to shared LIBRA resources" FORCE
)
