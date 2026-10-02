################################################################################
# @file   CMakeLists.txt
# @brief  Project modules root CMakeLists file.
#
# @author Christian Brice
################################################################################

# Add module subdirectories

# (1) Top-level dependencies (order matters!)
add_subdirectory(common)
add_subdirectory(util)
add_subdirectory(logger)

# (2) Project modules
add_subdirectory(arduino_controllers)
add_subdirectory(camera_manager)
add_subdirectory(motion_controller)
