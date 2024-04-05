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

    default_urdf_model_path = os.path.join(
        libra_pkg_share, 'models/camera.urdf')
    default_rviz_config_path = os.path.join(
        libra_pkg_share, 'rviz/rviz_basic_urg_settings.rviz')

    # --- DEFINE LAUNCH OPTIONS ---

    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start robot state publisher')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use simulation (Gazebo) clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RViz')

    # --- DEFINE ROS ACTIONS ---

    # Subscribe to and transform camera data
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': ParameterValue(Command(['xacro ', urdf_model]), value_type=str)}],
        arguments=[default_urdf_model_path])

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config])

    # --- DECLARE LAUNCH OPTIONS ---

    ld = LaunchDescription()

    # TODO

    # --- DECLARE ROS ACTIONS ---
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(start_rviz_cmd)

    return ld
