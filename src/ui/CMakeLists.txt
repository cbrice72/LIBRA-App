################################################################################
# @file   CMakeLists.txt
# @brief  Project user interface (UI) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Configure settings
set(RESOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/res)

# Add UI modules
add_subdirectory(dialogs)  # open_serial_dialog, torque_comp_dialog, (open_epos_dialog)
add_subdirectory(main_window)  # UI_SOURCE

# Propagate changes (only necessary if UI_SOURCE is appended to after creation)
set(UI_SOURCE
    ${UI_SOURCE}
    PARENT_SCOPE
)
