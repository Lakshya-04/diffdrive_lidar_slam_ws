#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch slam_toolbox for online mapping
    """

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mobile_robot_slam'),
            'config',
            'slam_toolbox_params.yaml'
        ]),
        description='Full path to the SLAM parameters file'
    )

    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        slam_node
    ])

# #!/usr/bin/env python3

# import os
# import time
# from threading import Thread

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import LifecycleNode, Node
# from launch_ros.substitutions import FindPackageShare
# import rclpy
# from lifecycle_msgs.srv import ChangeState

# def activate_slam_toolbox(context, *args, **kwargs):
#     """
#     Function to activate slam_toolbox after a delay
#     This runs in a separate thread to avoid blocking the launch
#     """
#     def activate():
#         # Give the node time to start
#         time.sleep(3)

#         # Initialize ROS2 client
#         rclpy.init()
#         node = rclpy.create_node('slam_activator')

#         # Create client for lifecycle service
#         configure_client = node.create_client(ChangeState, '/slam_toolbox/change_state')

#         # Wait for service
#         if configure_client.wait_for_service(timeout_sec=5.0):
#             # Configure request
#             configure_request = ChangeState.Request()
#             configure_request.transition.id = 1  # CONFIGURE

#             # Call configure
#             configure_future = configure_client.call_async(configure_request)
#             rclpy.spin_until_future_complete(node, configure_future)

#             if configure_future.result().success:
#                 # Activate request
#                 activate_request = ChangeState.Request()
#                 activate_request.transition.id = 3  # ACTIVATE

#                 # Call activate
#                 activate_future = configure_client.call_async(activate_request)
#                 rclpy.spin_until_future_complete(node, activate_future)

#                 if activate_future.result().success:
#                     node.get_logger().info("SLAM Toolbox activated successfully!")
#                 else:
#                     node.get_logger().error("Failed to activate SLAM Toolbox")
#             else:
#                 node.get_logger().error("Failed to configure SLAM Toolbox")

#         node.destroy_node()
#         rclpy.shutdown()

#     # Start activation in separate thread
#     activation_thread = Thread(target=activate)
#     activation_thread.daemon = True
#     activation_thread.start()

#     return []

# def generate_launch_description():
#     """
#     Launch slam_toolbox with simple activation thread
#     """

#     # Launch configuration variables
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     slam_params_file = LaunchConfiguration('slam_params_file')

#     # Declare launch arguments
#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use simulation/Gazebo clock'
#     )

#     declare_slam_params_file_cmd = DeclareLaunchArgument(
#         'slam_params_file',
#         default_value=PathJoinSubstitution([
#             FindPackageShare('mobile_robot_slam'),
#             'config',
#             'slam_toolbox_params.yaml'
#         ]),
#         description='Full path to SLAM parameters file'
#     )

#     # SLAM Toolbox lifecycle node
#     slam_toolbox_node = LifecycleNode(
#         parameters=[
#             slam_params_file,
#             {'use_sim_time': use_sim_time}
#         ],
#         package='slam_toolbox',
#         executable='sync_slam_toolbox_node',
#         name='slam_toolbox',
#         output='screen'
#     )

#     # Activation function (runs after node starts)
#     activation_action = OpaqueFunction(function=activate_slam_toolbox)

#     return LaunchDescription([
#         declare_use_sim_time_cmd,
#         declare_slam_params_file_cmd,
#         slam_toolbox_node,
#         activation_action  # This automatically activates after 3 seconds
#     ])

# Launch
# ros2 launch mobile_robot_bringup bringup.launch.py

# ros2 launch mobile_robot_slam slam.launch.py 

# # Activate the current sync node 
# ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
# ros2 service call /slam_toolbox/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"



