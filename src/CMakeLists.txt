################################################################################
# @file   CMakeLists.txt
# @brief  Project source root CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Collect project source files
add_subdirectory(ui)       # UI_SRC
add_subdirectory(widgets)  # WIDGET_SRC

# Define project sources for use by root CMakeLists.txt
set(PROJECT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${UI_SRC}
    ${WIDGET_SRC}
    PARENT_SCOPE
)
