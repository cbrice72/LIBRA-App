################################################################################
# @file   CMakeLists.txt
# @brief  Project widgets CMakeLists file.
#
# @author Christian Brice
################################################################################

# Specify subdirectory files for use by parent CMakeLists.txt
set(WIDGET_SOURCE
    # Fluid Tank Visualization
    ${CMAKE_CURRENT_SOURCE_DIR}/tank_widget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/tank_widget.cpp
    # Diagnostics Tabs
    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics_widget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics_widget.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics_widget.ui
    # Sideways Collapsible Section
    ${CMAKE_CURRENT_SOURCE_DIR}/collapsible_column.h
    ${CMAKE_CURRENT_SOURCE_DIR}/collapsible_column.cpp
    # Scrolling Line Graph
    ${CMAKE_CURRENT_SOURCE_DIR}/line_graph_widget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/line_graph_widget.cpp
    # 2D Dot Plot
    ${CMAKE_CURRENT_SOURCE_DIR}/dot_plot_widget.h
    ${CMAKE_CURRENT_SOURCE_DIR}/dot_plot_widget.cpp
    PARENT_SCOPE
)
