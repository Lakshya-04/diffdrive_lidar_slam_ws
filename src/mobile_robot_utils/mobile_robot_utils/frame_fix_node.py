#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FrameFixerNode(Node):
    def __init__(self):
        super().__init__('frame_fixer_node')
        self.sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/fixed_scan', 10)

    def listener_callback(self, msg):
        msg.header.frame_id = 'lidar_link'  # your correct frame_id here
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameFixerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
