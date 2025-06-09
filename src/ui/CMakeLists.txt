################################################################################
# @file   CMakeLists.txt
# @brief  Project user interface (UI) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Specify subdirectory files for use by parent CMakeLists.txt
set(UI_SOURCE
    # UI Resources (icons, etc.)
    ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
    # Main Window
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.h
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.ui
)

if(LIBRA_VERSION EQUAL 2)
    list(
        APPEND
        UI_SOURCE
        # EPOS Connection Dialog
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_window.h
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_window.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_window.ui
    )
endif()

set(UI_SOURCE ${UI_SOURCE} PARENT_SCOPE)  # propagate changes
