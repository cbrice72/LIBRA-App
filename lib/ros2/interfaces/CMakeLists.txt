################################################################################
# @file   CMakeLists.txt
# @brief  LIBRA ROS2 interfaces (message types, services, etc.) CMakeLists file.
#
# @author brice.c.aa
################################################################################

# List necessary files for each interface
set(actuator_msgs
    "msg/HebiState.msg"
)

# Generate all interfaces at once
rosidl_generate_interfaces(libra_interfaces
    ${actuator_msgs}
    DEPENDENCIES ${ROS_MSG_PACKAGES}
)

# --- The following jankiness seems to be required to use ROS2 objects in pure CMake ---

# Expose typesupport target and base name for linking in parent project
rosidl_get_typesupport_target(libra_interfaces_cpp_target libra_interfaces "rosidl_typesupport_cpp")
set(LIBRA_INTERFACES_CPP_TARGET ${libra_interfaces_cpp_target} PARENT_SCOPE)
