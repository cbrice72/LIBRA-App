################################################################################
# @file   CMakeLists.txt
# @brief  Project source root CMakeLists file.
#
# @author Christian Brice
################################################################################

# ========================= SETUP =========================

# Set common variables
set(RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ui/res")

# Add project modules
add_subdirectory(thread) # actuator_thread

# Collect project source files
add_subdirectory(ui) # UI_SOURCE, various dialogs
add_subdirectory(widgets) # WIDGET_SOURCE

# Validation
if(NOT DEFINED UI_SOURCE OR NOT DEFINED WIDGET_SOURCE)
    message(FATAL_ERROR "One or more required source variables are undefined! Check if subdirectories were built correctly")
endif()

set(PROJECT_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${UI_SOURCE} ${WIDGET_SOURCE}
)

# ========================= LIBRA APP GUI =========================

# Create the GUI application target
qt_add_executable(libra_app_gui MANUAL_FINALIZATION ${PROJECT_SOURCE})

# Ensure it has access to the necessary headers
target_include_directories(libra_app_gui PRIVATE ${CMAKE_SOURCE_DIR}/src ${RESOURCE_DIR})

# Tell the linker which libraries to link against
set(LIBRA_LIBRARIES arduino_manager camera_manager common logger util)

set(LIBRA_THREADS actuator_thread)

set(LIBRA_DIALOGS open_serial_dialog torque_comp_dialog)
if(LIBRA_VERSION EQUAL 2)
    list(APPEND LIBRA_DIALOGS open_epos_dialog)
endif()

target_link_libraries(
    libra_app_gui
    # Project
    PRIVATE ${LIBRA_LIBRARIES}
    PRIVATE ${LIBRA_THREADS}
    PRIVATE ${LIBRA_DIALOGS}
    # Qt
    PRIVATE Qt${QT_VERSION_MAJOR}::Core
    PRIVATE Qt${QT_VERSION_MAJOR}::Gui
    # ~~~
    # Compilation breaks if Multimedia and MultimediaWidgets aren't linked to
    # the main executable, even though they're only needed by camera_manager.
    # Why does this happen? I don't know. I'm tired.
    # ~~~
    PRIVATE Qt${QT_VERSION_MAJOR}::Multimedia
    PRIVATE Qt${QT_VERSION_MAJOR}::MultimediaWidgets
    PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
)

# Specify platform-specific properties
# ~~~
set_target_properties(
    libra_app_gui
    PROPERTIES ${BUNDLE_ID_OPTION} MACOSX_BUNDLE_BUNDLE_VERSION
               ${PROJECT_VERSION} MACOSX_BUNDLE_SHORT_VERSION_STRING
               ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR} MACOSX_BUNDLE
               TRUE WIN32_EXECUTABLE
               TRUE
)
# ~~~

# ~~~
# Define GUI target properties for Android with Qt 6 as:
#   set_target_properties(libra_app_gui
#       APPEND PROPERTY
#       QT_ANDROID_PACKAGE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/android
#   )
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
# ~~~

# Install the application
install(
    TARGETS libra_app_gui
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_custom_command( # copy runtime config files to output dir
    TARGET libra_app_gui
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIBRA_SHARED}
            ${CMAKE_INSTALL_BINDIR}/shared
)

qt_finalize_executable(libra_app_gui)
