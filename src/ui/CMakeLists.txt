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

# ========================= OPEN SERIAL DIALOG =========================

# Create and configure the module
qt_add_library(
    open_serial_dialog
    STATIC
    # UI Resources (icons, etc.)
    ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
    # Dialog
    ${CMAKE_CURRENT_SOURCE_DIR}/open_serial_dialog.h
    ${CMAKE_CURRENT_SOURCE_DIR}/open_serial_dialog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/open_serial_dialog.ui
)

# Ensure this module packages the necessary headers
target_include_directories(
    open_serial_dialog
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    open_serial_dialog
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Widgets
    PUBLIC Qt${QT_VERSION_MAJOR}::SerialPort
)

if(LIBRA_VERSION EQUAL 2)
    # ========================= OPEN EPOS DIALOG =========================

    # Create and configure the module
    qt_add_library(
        open_epos_dialog
        STATIC
        # UI Resources (icons, etc.)
        ${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc
        # Dialog
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_dialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_dialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_dialog.ui
    )

    # Ensure this module packages the necessary headers
    target_include_directories(
        open_epos_dialog
        # EPOS (Maxon)
        PUBLIC ${EPOS_INCLUDE_DIRS}
        # Make this directory available to consumers
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # Tell the linker which libraries to link against
    target_link_libraries(
        open_epos_dialog
        # Project
        PUBLIC util
        # EPOS (Maxon)
        PRIVATE -lEposCmd
        # Qt
        PUBLIC Qt${QT_VERSION_MAJOR}::Widgets
    )
endif()

# Propagate changes (only necessary if UI_SOURCE is appended to after creation)
set(UI_SOURCE
    ${UI_SOURCE}
    PARENT_SCOPE
)
