import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    description_pkg = get_package_share_directory('mobile_robot_description')
    gazebo_pkg = get_package_share_directory('mobile_robot_gazebo')
    utils_pkg = get_package_share_directory('mobile_robot_utils')

    model_path = os.path.join(description_pkg, 'model', 'robot.xacro')
    robot_description = xacro.process_file(model_path).toxml()

    bridge_params_path = os.path.join(gazebo_pkg, 'config', 'bridge_parameters.yaml')
    ekf_params_path = os.path.join(get_package_share_directory('mobile_robot_bringup'), 'config', 'ekf.yaml')
    # rviz_config_path = os.path.join(get_package_share_directory('mobile_robot_bringup'), 'config', 'mobile_robot_2dlidar.rviz')
    rviz_config_path = os.path.join(get_package_share_directory('mobile_robot_bringup'), 'config', 'mobile_robot_depth.rviz')

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gazebo_sim.launch.py'))
    )

    # Spawn robot
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diff_drive_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params_path}'],
        output='screen'
    )

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path, {'use_sim_time': True}]
    )

    # Utils
    frame_fix_node = Node(
        package='mobile_robot_utils',
        executable='frame_fix_node',
        name='frame_fix_node',
        output='screen'
    )

    odom_to_path_node = Node(
        package='mobile_robot_utils',
        executable='odom_to_path_node',
        name='odom_to_path_node',
        output='screen'
    )

    # RViz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        remappings=[('/scan', '/fixed_scan')]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_model_node,
        robot_state_publisher_node,
        bridge_node,
        ekf_node,
        frame_fix_node,
        odom_to_path_node,
        rviz2_node
    ])
