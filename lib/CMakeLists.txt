################################################################################
# @file   CMakeLists.txt
# @brief  Project modules root CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Add module subdirectories
# - Top-level dependencies
add_subdirectory(util)
add_subdirectory(logger)
# - "Manager"-type modules
add_subdirectory(camera_manager)
# - Optional modules
if(BUILD_WITH_ROS2)
    add_subdirectory(ros2)
endif()
