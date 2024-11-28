################################################################################
# @file   CMakeLists.txt
# @brief  Arm control thread module CMakeLists file.
#         Of the two libraries: `libra_actuators` defines types of actuators as
#         derivatives of a common template, and `arm_thread` implements them as
#         an interactive QThread object.
#
# @author brice.c.aa
################################################################################

# ---
# LIBRA Actuators
# ---

# EPOS Setup
# - Check whether the EPOS directory was defined
if(NOT DEFINED EPOS_DIR)
    message(FATAL_ERROR "EPOS_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable")
endif()
# - Include EPOS headers
set(EPOS_INCLUDE_DIRS "${EPOS_DIR}/include")

# HEBI Setup
# - Check whether the HEBI directory was defined
if(NOT DEFINED HEBI_DIR)
    message(FATAL_ERROR "HEBI_DIR is not defined! Set it in your root CMakeLists.txt as a cached variable")
endif()
# - Collect all HEBI source files (see comment in `qt_add_library` call below)
file(GLOB HEBI_SOURCE_FILES "${HEBI_DIR}/src/*.cpp")
# - Include headers from all libraries used by HEBI (TODO: is this overkill?)
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/Eigen" "${HEBI_DIR}/hebi/include"
                      "${HEBI_DIR}/rapidyaml" "${HEBI_DIR}/src"
)

# Create and configure the module
qt_add_library(
    libra_actuators
    STATIC
    # Header
    abstract_actuator.h
    epos_actuator.h
    hebi_actuator.h
    # Source
    epos_actuator.cpp
    hebi_actuator.cpp
    # HEBI src/ folder needs to be passed in during compilation/linking process
    # (see HEBI C++ API docs: https://files.hebi.us/docs/cpp/cpp-3.11.1/)
    ${HEBI_SOURCE_FILES}
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    libra_actuators
    # EPOS (Maxon)
    PUBLIC ${EPOS_INCLUDE_DIRS}
    # HEBI
    PUBLIC ${HEBI_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    libra_actuators
    # EPOS (Maxon)
    PRIVATE -lEposCmd
    # HEBI
    PRIVATE pthread hebic++
)

# ---
# Arm Thread
# ---

# Create and configure the module
qt_add_library(
    arm_thread
    STATIC
    # Header
    arm_thread.h
    # Source
    arm_thread.cpp
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    arm_thread
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    arm_thread
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Core
)
