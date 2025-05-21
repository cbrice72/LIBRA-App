################################################################################
# @file   CMakeLists.txt
# @brief  Project user interface (UI) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Specify subdirectory files for use by parent CMakeLists.txt
set(UI_SOURCE
    # UI Resources (icons, etc.)
    # ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
    # Main Window
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.h
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.ui
    PARENT_SCOPE
)
