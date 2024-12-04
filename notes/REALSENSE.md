# Intel RealSense Camera + ROS2

This document lists the necessary steps to set up and use the `realsense2_camera` ROS2 nodes in a Linux environment.

## Before You Start

Follow the instructions in the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) git repo to download and build the project.

## Usage

### *ROS Nodes*

#### **Terminal 1**

```bash
# Enable "accel" and "gyro" publishing (note: these can also be combined into an "imu" msg; see RealSense node documentation)
# Enable "pointcloud" post-processing filter
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true enable_gyro:=true enable_accel:=true
```

#### **Terminal 2**

```bash
# Run RViz and configure to show Image, DepthCloud, PointCloud2 visualizations.
rviz2
```

## Troubleshooting

(none)

## Optional Items

(none)
