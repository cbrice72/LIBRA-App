################################################################################
# @file   CMakeLists.txt
# @brief  Project modules root CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Add module subdirectories
# - Top-level dependencies (order matters!)
add_subdirectory(common)
add_subdirectory(util)
add_subdirectory(logger)
# - "Manager"-type modules
add_subdirectory(arduino_manager)
add_subdirectory(camera_manager)
