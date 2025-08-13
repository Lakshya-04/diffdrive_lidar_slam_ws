#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdometryToPathNode(Node):
    """
    A ROS 2 node that subscribes to Odometry messages, accumulates them,
    and publishes them as Path messages for visualization.
    """
    def __init__(self):
        super().__init__('odom_to_path_node')

        # Declare a parameter for the maximum number of poses to store in the path
        self.declare_parameter('path_length', 1000)
        self.max_poses = self.get_parameter('path_length').get_parameter_value().integer_value

        # --- Raw Odometry Path ---
        self.raw_path = Path()
        # Use a simple QoS setting that is more compatible
        self.raw_odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.raw_odom_callback,
            10  # Use default QoS with a history depth of 10
        )
        self.raw_path_pub = self.create_publisher(Path, '/trajectory/raw', 10)

        # --- Filtered Odometry Path ---
        self.filtered_path = Path()
        # Use a simple QoS setting that is more compatible
        self.filtered_odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.filtered_odom_callback,
            10  # Use default QoS with a history depth of 10
        )
        self.filtered_path_pub = self.create_publisher(Path, '/trajectory/filtered', 10)
        
        self.get_logger().info(f"Odometry to Path converter node started. Path length set to {self.max_poses}.")

    def raw_odom_callback(self, msg: Odometry):
        """Callback to process raw odometry and publish as a path."""
        self.update_path(msg, self.raw_path, self.raw_path_pub)
        
    def filtered_odom_callback(self, msg: Odometry):
        """Callback to process filtered odometry and publish as a path."""
        self.update_path(msg, self.filtered_path, self.filtered_path_pub)

    def update_path(self, odom_msg: Odometry, path_msg: Path, publisher):
        """
        Updates a Path message with a new pose from an Odometry message
        and publishes the updated path.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = odom_msg.header
        pose_stamped.pose = odom_msg.pose.pose
        
        path_msg.poses.append(pose_stamped)
        
        if len(path_msg.poses) > self.max_poses:
            path_msg.poses.pop(0)
            
        path_msg.header = odom_msg.header
        publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()