import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Set paths to required files and dirs
    pkg_share = FindPackageShare(package='libra').find('libra')
    default_rviz_config_path = os.path.join(
        pkg_share, 'rviz/rviz_basic_settings.rviz')
    default_urdf_model_path = os.path.join(
        pkg_share, 'models/sensor_suite.urdf')

    # --- DEFINE LAUNCH OPTIONS ---

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to RViz config file')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Full path to robot URDF file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use simulation (Gazebo) clock')

    # --- DEFINE ROS ACTIONS ---

    # Publish joint state values (see URDF)
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    # Subscribe to joint states of robot and publish 3D pose of each link
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     # Note: If ANY text in the xacro output can be interpreted as yaml,
                     # the roslaunch system will try to interpret the ENTIRE text as yaml
                     # instead of passing on the string. The biggest cause of this false
                     # interpretation is commenting out xacro calls since the xacro:property
                     # or similar looks a lot like a yaml key:value pair. Comments are not
                     # removed by xacro so they are included in the output.
                     # (source: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/)
                     'robot_description': ParameterValue(Command(['xacro ', urdf_model]), value_type=str)}],
        arguments=[default_urdf_model_path])

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    # --- DECLARE LAUNCH OPTIONS ---

    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # --- DECLARE ROS ACTIONS ---

    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
