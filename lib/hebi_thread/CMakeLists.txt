################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA HEBI actuators module CMakeLists file.
#
# @author brice.c.aa
# @date   2024/2/22
################################################################################

# Specify the HEBI library directory
set(HEBI_DIR ${CMAKE_SOURCE_DIR}/thirdparty/hebi-cpp-3.7.0/hebi-cpp)

# Collect all HEBI source files (see comment in `qt_add_library` call below)
file(GLOB HEBI_SOURCE_FILES "${HEBI_DIR}/src/*.cpp")

# Create and configure the module
qt_add_library(
    hebi_thread
    STATIC
    # Header
    hebi_thread.h
    # Source
    hebi_thread.cpp
    # HEBI src/ folder needs to be passed in during compilation/linking process
    # (see HEBI C++ API docs: https://files.hebi.us/docs/cpp/cpp-3.7.0/)
    ${HEBI_SOURCE_FILES}
)

# Ensure this module has access to the necessary 3rd-party libraries
set(HEBI_INCLUDE_DIRS "${HEBI_DIR}/hebi/include" "${HEBI_DIR}/src")

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    hebi_thread
    # HEBI
    PUBLIC ${HEBI_INCLUDE_DIRS}
    # Make sure the path to this directory is automatically added as an include
    # path to all targets using this library
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Import the HEBI library packaged with this project
# ~~~
# Note: if the versioned library in the set_target_properties() call below is
# changed (e.g., updated), make sure to update the .gitignore entry too!
# ~~~
add_library(hebi SHARED IMPORTED GLOBAL)
set_target_properties(
    hebi PROPERTIES IMPORTED_LOCATION ${HEBI_DIR}/hebi/lib/libhebi.so.2.9
)

# Tell the linker which libraries to link against
target_link_libraries(
    hebi_thread
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    # HEBI
    PRIVATE pthread hebi
)

# Create symbolic links for unversioned HEBI library shared object (.so) files
# ~~~
# For some reason, the unversioned library files that come in the HEBI C++ API
# download don't work as symbolic links, so we have to generate them ourselves.
# ~~~
add_custom_command(
    TARGET hebi_thread
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            ${HEBI_DIR}/hebi/lib/libhebi.so.2.9 ${HEBI_DIR}/hebi/lib/libhebi.so
)
add_custom_command(
    TARGET hebi_thread
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E create_symlink ${HEBI_DIR}/hebi/lib/libhebi.so.2.9
        ${HEBI_DIR}/hebi/lib/libhebi.so.2
)
