################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA ROS2 nodes CMakeLists file.
#
# @author brice.c.aa
################################################################################

# ========================= MINIMAL PUB/SUB NODE =========================

# Create and configure the module
add_executable(
    minimal_pub_sub
    # Source
    ${CMAKE_CURRENT_SOURCE_DIR}/src/minimal_pub_sub.cpp
)

set_target_properties(
    minimal_pub_sub PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Ensure this module has access to the necessary 3rd-party libraries
target_include_directories(
    minimal_pub_sub
    # Make this directory available to consumers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(minimal_pub_sub PUBLIC rclcpp std_msgs)

# Specify installation settings
install(TARGETS minimal_pub_sub)
