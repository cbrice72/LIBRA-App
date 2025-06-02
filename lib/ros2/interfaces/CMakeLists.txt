################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA ROS2 interfaces (message types, services, etc.) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# List necessary files for each interface
set(actuator_msgs
    # HEBI Actuator
    "msg/HebiState.msg"
)

# Generate all interfaces at once
rosidl_generate_interfaces(
    ${PROJECT_NAME} ${actuator_msgs} DEPENDENCIES builtin_interfaces std_msgs
)

# --- The following seems to be required to use ROS2 objects in pure CMake ---

# Expose typesupport target and generator dir for linking
rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
set(LIBRA_INTERFACES_CPP_TARGET
    ${cpp_typesupport_target}
    CACHE STRING "Compilation target for all LIBRA message interfaces" FORCE
)
set(LIBRA_INTERFACES_INCLUDE_DIR
    "${CMAKE_BINARY_DIR}/lib/ros2/interfaces/rosidl_generator_cpp"
    CACHE PATH "Path to the directory containing generated ROS2 messages" FORCE
)
