################################################################################
# @file   Ros2Setup.cmake
# @brief  CMake module for validating and loading ROS2 dependencies.
#
# @author brice.c.aa
################################################################################

# ========================= VALIDATION =========================

if(NOT DEFINED ENV{AMENT_PREFIX_PATH})
    message(FATAL_ERROR
        "AMENT_PREFIX_PATH is not set; ROS2 environment was not sourced.\n"
        "Run: source /opt/ros/<distro>/setup.bash"
    )
endif()

if(NOT "$ENV{ROS_VERSION}" STREQUAL "2")
    message(FATAL_ERROR "ROS_VERSION is not 2 — no valid ROS2 installation found.")
endif()

# ========================= DEPENDENCIES =========================

# Core libraries
set(ROS_CORE_PACKAGES
    ament_cmake
    rclcpp
    rcutils
    rmw_implementation
    rosidl_default_generators
)

foreach(PKG ${ROS_CORE_PACKAGES})
    find_package(${PKG} REQUIRED)
endforeach()

# Messages and interfaces
set(ROS_MSG_PACKAGES
    builtin_interfaces
    std_msgs
    sensor_msgs
)

foreach(PKG ${ROS_MSG_PACKAGES})
    find_package(${PKG} REQUIRED)
endforeach()
