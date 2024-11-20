################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA arm actuators module CMakeLists file.
#
# @author brice.c.aa
################################################################################

# TODO: add EPOS actuators

# Collect all HEBI source files (see comment in `qt_add_library` call below)
file(GLOB HEBI_SOURCE_FILES "${HEBI_DIR}/src/*.cpp")

# Create and configure the module
qt_add_library(
    arm_thread
    STATIC
    # Header
    arm_thread.h
    # Source
    arm_thread.cpp
    # HEBI src/ folder needs to be passed in during compilation/linking process
    # (see HEBI C++ API docs: https://files.hebi.us/docs/cpp/cpp-3.11.1/)
    ${HEBI_SOURCE_FILES}
)

# Ensure this module has access to the necessary 3rd-party libraries
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/Eigen" "${HEBI_DIR}/hebi/include"
                      "${HEBI_DIR}/rapidyaml" "${HEBI_DIR}/src"
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    arm_thread
    # HEBI
    PUBLIC ${HEBI_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    arm_thread
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    # HEBI
    PRIVATE pthread hebic++
)
