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
    libra_pkg_share = FindPackageShare(package='libra').find('libra')
    urg_pkg_share = FindPackageShare(package='urg_node').find('urg_node')

    default_sensor_config_path = os.path.join(
        urg_pkg_share, 'launch/urg_node_serial.yaml')
    default_urdf_model_path = os.path.join(
        urg_pkg_share, 'launch/hokuyo_laser.urdf')
    default_rviz_config_path = os.path.join(
        libra_pkg_share, 'rviz/rviz_basic_urg_settings.rviz')

    # --- DEFINE LAUNCH OPTIONS ---

    sensor_config = LaunchConfiguration('sensor_config')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config = LaunchConfiguration('rviz_config')
    use_urg_driver = LaunchConfiguration('use_urg_driver')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
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

    declare_use_urg_driver_cmd = DeclareLaunchArgument(
        name='use_urg_driver',
        default_value='True',
        description='Whether to start URG driver')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RViz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Whether to use simulation (Gazebo) clock')

    # --- DEFINE ROS ACTIONS ---

    # Launch LIDAR driver to collect data
    start_urg_driver_cmd = Node(
        condition=IfCondition(use_urg_driver),
        package='urg_node',
        executable='urg_node_driver',
        name='urg_driver',
        arguments=['--ros-args', '--params-file', sensor_config]
        # parameters=[LaunchConfiguration('sensor_config')]
    )

    # Subscribe to and transform LIDAR data
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
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
        condition=IfCondition(use_rviz),
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
    ld.add_action(declare_use_urg_driver_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # --- DECLARE ROS ACTIONS ---

    ld.add_action(start_urg_driver_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
