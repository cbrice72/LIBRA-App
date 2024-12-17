################################################################################
# @file   CMakeLists.txt
# @brief  Project modules root CMakeLists file.
#
# @author brice.c.aa
################################################################################

# Add module subdirectories
add_subdirectory(camera_manager)
add_subdirectory(libra_hebi)  # TODO: finish hebi_thread
#add_subdirectory(lidar_hokuyo)
add_subdirectory(util)  # TODO(brice.c.aa): add EPOS actuator
