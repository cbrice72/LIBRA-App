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
    PARENT_SCOPE
)
