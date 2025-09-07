import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Enhanced Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='ground.sdf',
        description='World file name (must exist in mobile_robot_gazebo/worlds/ folder)'
    )
    
    lidar_type_arg = DeclareLaunchArgument(
        'lidar_type',
        default_value='2d',
        description='LiDAR type: 2d, 3d, or depth'
    )
    
    use_rgbd_arg = DeclareLaunchArgument(
        'use_rgbd',
        default_value='true',
        description='Enable RGBD camera'
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
    
    # Dynamic RViz configs
    rviz_config_2d = os.path.join(bringup_pkg, 'config', 'mobile_robot_2dlidar.rviz')
    rviz_config_3d = os.path.join(bringup_pkg, 'config', 'mobile_robot_3dlidar.rviz')
    rviz_config_depth = os.path.join(bringup_pkg, 'config', 'mobile_robot_depth.rviz')
    
    # World file path
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

    # Robot nodes
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diff_drive_robot', '-topic', 'robot_description'],
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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path, {'use_sim_time': True}]
    )

    # Conditional nodes based on sensor configuration
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

    # Static transforms for different sensors
    rgbd_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgbd_camera_tf',
        arguments=['0', '0', '0', '0', '0', '0', 
                'rgbd_camera_link', 'diff_drive_robot/base_footprint/rgbd_camera_sensor'],
        output='screen'
    )

    # ADDITIONAL: Add optical frame transform
    rgbd_camera_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgbd_camera_optical_tf', 
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                'rgbd_camera_link', 'rgbd_camera_optical_link'],
        output='screen'
    )

    # RViz configurations - select config based on lidar_type
    rviz2_node_2d = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_2d],
        remappings=[('/scan', '/fixed_scan')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lidar_type'), "' == '2d'"]))
    )
    rviz2_node_3d = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_3d],
        remappings=[('/scan', '/fixed_scan')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lidar_type'), "' == '3d'"]))
    )
    rviz2_node_depth = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_depth],
        remappings=[('/scan', '/fixed_scan')],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('lidar_type'), "' == 'depth'"]))
    )

    # 3D LiDAR static transform only if lidar_type is 3d or depth
    lidar_3d_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_3d_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                  'lidar_3d_link', 'diff_drive_robot/base_footprint/gpu_lidar_3d'],
        output='screen',
        condition=IfCondition(PythonExpression([
            "('", LaunchConfiguration('lidar_type'), "' == '3d') or ('", LaunchConfiguration('lidar_type'), "' == 'depth')"
        ]))
    )

    # Interactive Marker
    interactive_marker_twist_node = Node(
        package='mobile_robot_utils',
        executable='interactive_marker_twist',
        name='interactive_marker_twist_server',
        output='screen'
    )

    # PCL Processing Node
    pcl_processor_node = Node(
        package='mobile_robot_slam',
        executable='pcl_processor',
        name='pcl_processor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        lidar_type_arg,
        use_rgbd_arg,
        use_rviz_arg,
        gui_arg,
        gazebo_launch,
        spawn_model_node,
        robot_state_publisher_node,
        bridge_node,
        ekf_node,
        frame_fix_node,
        rgbd_camera_tf_node,
        rgbd_camera_optical_tf_node,
        lidar_3d_tf_node,
        odom_to_path_node,
        rviz2_node_2d,
        rviz2_node_3d,
        rviz2_node_depth,
        interactive_marker_twist_node,
        pcl_processor_node
    ])
