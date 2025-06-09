################################################################################
# @file   CMakeLists.txt
# @brief  Utility module CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Create and configure the module
add_library(
    util
    STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/util.h
    ${CMAKE_CURRENT_SOURCE_DIR}/util.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    util
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    util
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
)

if(LIBRA_VERSION EQUAL 2)
    # Additional includes and libraries for the EPOS actuator
    set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")

    target_include_directories(
        util
        # EPOS (Maxon)
        PRIVATE ${EPOS_INCLUDE_DIRS}
    )

    target_link_libraries(
        util
        # EPOS (Maxon)
        PRIVATE -lEposCmd
    )
endif()
