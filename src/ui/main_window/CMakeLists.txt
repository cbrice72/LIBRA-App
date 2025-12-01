################################################################################
# @file   CMakeLists.txt
# @brief  Project UI main window CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Specify subdirectory files for use by parent CMakeLists.txt
set(UI_SOURCE
    # UI Resources (icons, etc.)
    ${RESOURCE_DIR}/colors.h
    ${RESOURCE_DIR}/resources.qrc
    # Main Window
    ${CMAKE_CURRENT_SOURCE_DIR}/arm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.h
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main_window.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/menu_bar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/water.cpp
)

if(LIBRA_VERSION EQUAL 1)
    list(APPEND UI_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/manipulator.cpp
    )
endif()

# Propagate changes (only necessary if UI_SOURCE is appended to after creation)
set(UI_SOURCE
    ${UI_SOURCE}
    PARENT_SCOPE
)
