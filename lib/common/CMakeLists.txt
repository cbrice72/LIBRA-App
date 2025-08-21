################################################################################
# @file   CMakeLists.txt
# @brief  Common project definitions CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Create and configure the module
set(SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/actuator_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/joint_defs.h
)

add_library(common INTERFACE ${SRC_FILES})

# Ensure this module packages the necessary headers
target_include_directories(
    common
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)
