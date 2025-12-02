################################################################################
# @file   CMakeLists.txt
# @brief  Project UI dialog windows CMakeLists file.
#
# @author brice.c.aa
################################################################################

if(LIBRA_VERSION EQUAL 2)
    # ========================= OPEN EPOS DIALOG =========================

    # Create and configure the module
    qt_add_library(
        open_epos_dialog
        STATIC
        # UI Resources (icons, etc.)
        ${RESOURCE_DIR}/resources.qrc
        # Dialog
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_dialog.h
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_dialog.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/open_epos_dialog.ui
    )

    # Ensure this module packages the necessary headers
    target_include_directories(
        open_epos_dialog
        # Project
        PUBLIC ${RESOURCE_DIR}
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

# ========================= OPEN SERIAL DIALOG =========================

# Create and configure the module
qt_add_library(
    open_serial_dialog
    STATIC
    # UI Resources (icons, etc.)
    ${RESOURCE_DIR}/resources.qrc
    # Dialog
    ${CMAKE_CURRENT_SOURCE_DIR}/open_serial_dialog.h
    ${CMAKE_CURRENT_SOURCE_DIR}/open_serial_dialog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/open_serial_dialog.ui
)

# Ensure this module packages the necessary headers
target_include_directories(
    open_serial_dialog
    # Project
    PUBLIC ${RESOURCE_DIR}
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

# ========================= TORQUE COMP DIALOG =========================

# Create and configure the module
qt_add_library(
    torque_comp_dialog
    STATIC
    # UI Resources (icons, etc.)
    ${RESOURCE_DIR}/resources.qrc
    # Dialog
    ${CMAKE_CURRENT_SOURCE_DIR}/torque_comp_dialog.h
    ${CMAKE_CURRENT_SOURCE_DIR}/torque_comp_dialog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/torque_comp_dialog.ui
)

# Ensure this module packages the necessary headers
target_include_directories(
    torque_comp_dialog
    # Project
    PUBLIC ${RESOURCE_DIR}
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Tell the linker which libraries to link against
target_link_libraries(
    torque_comp_dialog
    # Qt
    PUBLIC Qt${QT_VERSION_MAJOR}::Widgets
)
