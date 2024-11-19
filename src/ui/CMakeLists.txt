################################################################################
# @file   CMakeLists.txt
# @brief  Project user interface (UI) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Specify subdirectory files for use by parent CMakeLists.txt
set(UI_SRC
    # UI Resources (icons, etc.)
    ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
    # Main Window
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.h
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.ui
    # Serial (USB) Selection Dialog
    ${CMAKE_CURRENT_SOURCE_DIR}/serial_dialog.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serial_dialog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/serial_dialog.ui
    # EPOS Connection Dialog
    ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_window.h
    ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_window.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_window.ui
    PARENT_SCOPE
)