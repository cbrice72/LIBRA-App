################################################################################
# @file   CMakeLists.txt
# @brief  Project modules root CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Add module subdirectories
add_subdirectory(camera_manager)
add_subdirectory(logger)
add_subdirectory(util)
if(BUILD_WITH_ROS2)
    add_subdirectory(ros2)
endif()
