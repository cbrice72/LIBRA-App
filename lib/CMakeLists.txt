################################################################################
# @file   CMakeLists.txt
# @brief  Project modules root CMakeLists file.
#
# @author brice.c.aa
# @date   2024/2/13
################################################################################

# Add module subdirectories
add_subdirectory(libra_hebi)
add_subdirectory(libra_lidar)  # TODO(brice.c.aa): add Hokuyo LIDAR
add_subdirectory(serial)
#add_subdirectory(util)  # TODO(brice.c.aa): add EPOS actuator
