import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='ground.sdf',
        description='World file name (without path, must exist in worlds/ folder)'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true for verbose output'
    )   
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run headless'
    )
    
    # Build world file path directly from worlds folder
    gazebo_pkg = get_package_share_directory('mobile_robot_gazebo')
    world_file = PathJoinSubstitution([
        gazebo_pkg, 
        'worlds', 
        LaunchConfiguration('world')
    ])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                '-r -v ', 
                LaunchConfiguration('verbose'),
                ' ',
                ['--headless-rendering -s' if LaunchConfiguration('gui') == 'false' else ''],
                ' ',
                world_file
            ],
            'on_exit_shutdown': 'true'
        }.items()
    )

    return LaunchDescription([
        world_arg,
        verbose_arg,
        gui_arg,
        gazebo_launch
    ])
