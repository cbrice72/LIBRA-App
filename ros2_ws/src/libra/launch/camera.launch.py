import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# NOTE: see the following forum post for explanation on why ParameterValue() is used when passing "xacro" files as parameters to ROS actions.
# https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Set paths to required files and dirs
    libra_pkg_share = FindPackageShare(package='libra').find('libra')

    default_sensor_config_path = os.path.join(
        libra_pkg_share, 'config/camera_params.yaml')
    default_urdf_model_path = os.path.join(
        libra_pkg_share, 'models/camera.urdf')
    default_rviz_config_path = os.path.join(
        libra_pkg_share, 'rviz/rviz_basic_camera_settings.rviz')

    # --- DEFINE LAUNCH OPTIONS ---

    sensor_config = LaunchConfiguration('sensor_config')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sensor_config_file_cmd = DeclareLaunchArgument(
        name='sensor_config',
        default_value=default_sensor_config_path,
        description='Full path to sensor config file')

    declare_urdf_model_file_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Full path to URDF model file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to RViz config file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use simulation (Gazebo) clock')

    # --- DEFINE ROS ACTIONS ---

    # USB camera data
    start_camera_driver_cmd = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        arguments=['--ros-args', '--params-file', sensor_config]
    )

    # Robot data
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': ParameterValue(Command(['xacro ', urdf_model]), value_type=str)}],
        arguments=[default_urdf_model_path])

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config])

    # --- DECLARE LAUNCH OPTIONS ---

    ld = LaunchDescription()

    ld.add_action(declare_sensor_config_file_cmd)
    ld.add_action(declare_urdf_model_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # --- DECLARE ROS ACTIONS ---

    ld.add_action(start_camera_driver_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
