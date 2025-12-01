################################################################################
# @file   CMakeLists.txt
# @brief  Project user interface (UI) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Add UI modules
add_subdirectory(dialogs)  # open_serial_dialog, torque_comp_dialog, (open_epos_dialog)
add_subdirectory(main_window)  # UI_SOURCE

# Propagate source files upstream
set(UI_SOURCE
    ${UI_SOURCE}
    PARENT_SCOPE
)
