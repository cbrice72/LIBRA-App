# ROS2 Nodes leveraging the Intel RealSense Camera

## Official Intel RealSense Wrapper

GitHub repo: [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

### *Installation*

Just follow the instructions in the repo.
I didn't run into any particular problems.

### *Usage*

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
