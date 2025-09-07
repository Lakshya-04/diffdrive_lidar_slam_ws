import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='ground.sdf',
        description='World file name (must exist in mobile_robot_gazebo/worlds/ folder)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    # Package paths
    description_pkg = get_package_share_directory('mobile_robot_description')
    gazebo_pkg = get_package_share_directory('mobile_robot_gazebo')
    bringup_pkg = get_package_share_directory('mobile_robot_bringup')
    
    # File paths
    model_path = os.path.join(description_pkg, 'model', 'robot.xacro')
    robot_description = xacro.process_file(model_path).toxml()
    
    bridge_params_path = os.path.join(gazebo_pkg, 'config', 'bridge_parameters.yaml')
    ekf_params_path = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    # rviz_config_path = os.path.join(bringup_pkg, 'config', 'mobile_robot_depth.rviz')
    rviz_config_path = os.path.join(bringup_pkg, 'config', 'mobile_robot_2dlidar.rviz')
    
    # World file path - correctly reference the gazebo package
    world_file = PathJoinSubstitution([gazebo_pkg, 'worlds', LaunchConfiguration('world')])

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
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

    depth_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_tf',
        arguments=['0', '0', '0', '0', '0', '0', 
                'depth_camera_link', 'diff_drive_robot/base_footprint/rgbd_camera_sensor'],
        output='screen'
    )

    # RViz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        remappings=[('/scan', '/fixed_scan')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Interactive Marker Twist
    interactive_marker_twist_node = Node(
        package='mobile_robot_utils',
        executable='interactive_marker_twist',
        name='interactive_marker_twist_server',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        use_rviz_arg,
        gui_arg,
        gazebo_launch,
        spawn_model_node,
        robot_state_publisher_node,
        bridge_node,
        ekf_node,
        frame_fix_node,
        depth_camera_tf_node,
        odom_to_path_node,
        rviz2_node,
        interactive_marker_twist_node
    ])
