# ROS2 Nodes Tested for the LIBRA Project

## Intel RealSense

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

<br>

---

<br>

## ORB-SLAM3 ROS2 Wrapper

GitHub repo: [ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2/tree/humble)

### *Installation*

1. Follow instructions in "Prerequisites".
    - I also had to manually build Sophus before the next step (see "Troubleshootings")

2. Follow instructions in "How to build"; if build fails because of some OpenCV symbol missing, see note below.
    - In "2.", this is the Python library installed by ROS2 (path is similar to what is already written in the file)
    - In "3.", this is where you cloned the ORB_SLAM3 source code to in the previous step

> **Note:** if you get an error like `libopencv_XXX: error adding symbols: DSO missing from command line`, it simply means that a certain OpenCV library wasn't properly linked against (e.g., "libopencv_calib3d.so.4.5d", where "calib3d" is the library in question). In your `colcon_ws/src/orbslam3_ros2/CMakeLists.txt`, just add the following under the `link_directories` call: `link_libraries(-lopencv_calib3d)`

### *Usage*

1. Make an OpenCV-style calibration parameter config (`.yaml`) for your camera.
    - Most information can be found via `rs-enumerate-device -c`
    - The "baseline" can be found in the RealSense datasheet

#### **ORB-SLAM3 Standalone**

2. Run the stereo-inertial RealSense D435i example with the D456 config.
```bash
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D456.yaml 
```

> **Note:** enabling "Localization Mode" seems to cause the app to crash.

#### **ORB-SLAM3-ROS2**

2. Source your ROS2 workspace.
```bash
# Usually
. install/setup.bash
# Alternatively (from the ORB-SLAM3-ROS2 `README.md`)
/ install/local_setup.bash
```

3. Run in stereo-inertial mode. (TODO: doesn't work?)
```bash
# Standard command has the following syntax:
#   ros2 run orbslam3 <MODE> <PATH_TO_VOCAB> <PATH_TO_CONFIG>
#   - "STEREO" mode adds a required param: <BOOL_RECTIFY>
#   - "INERTIAL" mode further adds an optional param: [BOOL_EQUALIZE]
ros2 run orbslam3 stereo-inertial src/orbslam3_ros2/vocabulary/ORBvoc.txt src/orbslam3_ros2/config/stereo-inertial/RealSense_D456.yaml false
```
