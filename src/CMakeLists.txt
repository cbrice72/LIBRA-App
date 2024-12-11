################################################################################
# @file   CMakeLists.txt
# @brief  Project source root CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Collect project source files
add_subdirectory(thread)   # THREAD_SOURCE
add_subdirectory(ui)       # UI_SOURCE
add_subdirectory(widgets)  # WIDGET_SOURCE

# Define project sources for use by root CMakeLists.txt
set(PROJECT_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${THREAD_SOURCE}
    ${UI_SOURCE}
    ${WIDGET_SOURCE}
    PARENT_SCOPE
)
