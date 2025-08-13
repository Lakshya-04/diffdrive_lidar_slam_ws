import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    robotXacroName = 'diff_drive_robot'
    namePackage = 'mobile_robot'

    # --- File Paths ---
    pkg_share = get_package_share_directory(namePackage)
    model_path = os.path.join(pkg_share, 'model/robot.xacro')
    robot_description = xacro.process_file(model_path).toxml()
    bridge_params_path = os.path.join(pkg_share, 'parameters', 'bridge_parameters.yaml')
    ekf_params_path = os.path.join(pkg_share, 'config', 'ekf.yaml') # Path to your new EKF config
    rviz_config_path = os.path.join(pkg_share, 'config', 'mobile_robot_2dlidar.rviz')

    # --- Gazebo Simulation ---
    # Include Custom World
    # gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments= {'gz_args': ['-r -v -v4 CustomWorld.sdf'], 'on_exit_shutdown':'true'}.items())
    
    gazebo_ros_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_ros_launch,
        launch_arguments={'gz_args': '-r -v 4 empty.sdf', 'on_exit_shutdown': 'true'}.items()
    )

    # --- Nodes ---
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robotXacroName, '-topic', 'robot_description'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params_path}'],
        output='screen'
    )
    
    # --- EKF Node for Sensor Fusion ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path, {'use_sim_time': True}]
    )

    frameFixNode = Node(
        package=namePackage,
        executable='frame_fix_node',
        name='frame_fix_node',
        output='screen'
    )

    odom_to_path_node = Node(
        package='mobile_robot',
        executable='odom_to_path_node',
        name='odom_to_path_node',
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        remappings=[('/scan', '/fixed_scan')]
    )

    ld = LaunchDescription()
    
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_model_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(bridge_node)
    ld.add_action(ekf_node) # Add the new EKF node to the launch description
    ld.add_action(frameFixNode)
    ld.add_action(odom_to_path_node)
    ld.add_action(rviz2_node)

    return ld

