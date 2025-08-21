################################################################################
# @file   CMakeLists.txt
# @brief  Internal ROS2 workspace CMakeLists file.
#         Any targets built in these subdirectories will NOT be installed in the
#         same way that `colcon build` installs targets, so they can only be
#         used within the `libra_app` CMake project.
#
# @author brice.c.aa
################################################################################

# Register interfaces (e.g., messages) before the nodes that use them
add_subdirectory(interfaces)  # libra_interfaces (unused)
add_subdirectory(nodes)  # minimal_pub_sub
