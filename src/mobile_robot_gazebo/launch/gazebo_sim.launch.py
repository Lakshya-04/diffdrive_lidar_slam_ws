import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_ros_launch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    gazebo_launch = IncludeLaunchDescription(
        gazebo_ros_launch,
        launch_arguments={'gz_args': '-r -v 4 empty.sdf', 'on_exit_shutdown': 'true'}.items()
    )

    return LaunchDescription([gazebo_launch])
