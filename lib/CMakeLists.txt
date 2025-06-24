################################################################################
# @file   CMakeLists.txt
# @brief  Project modules root CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Add module subdirectories (in dependency order)
add_subdirectory(util)
add_subdirectory(logger)
add_subdirectory(camera_manager)
if(BUILD_WITH_ROS2)
    add_subdirectory(ros2)
endif()
