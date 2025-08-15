import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import Twist, Pose
import math

class InteractiveMarkerTwist(Node):
    def __init__(self):
        super().__init__('interactive_marker_twist_server')

        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create an interactive marker server
        self.server = InteractiveMarkerServer(
            self,
            "cmd_vel_marker"
        )

        self.create_marker()
        self.get_logger().info("Interactive Marker server is ready. Find the marker in RViz2.")

    def create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_footprint"  # Make sure this frame exists in your TF tree
        int_marker.name = "cmd_vel_marker"
        int_marker.description = "Move Robot"
        int_marker.pose.orientation.w = 1.0

        # Create a box marker for visualization
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.45
        box_marker.color.r = 0.5
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # Create a control that contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # --- Linear X control (move forward/backward) ---
        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.orientation.w = 1.0
        move_control.orientation.x = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(move_control)

        # --- Angular Z control (rotate) ---
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_z"
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.y = 1.0  # Changed from y to z for yaw rotation
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotate_control)

        # Insert the marker and set callback
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        twist = Twist()

        # While the user is dragging the marker, publish velocity commands
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Linear velocity based on x position
            twist.linear.x = feedback.pose.position.x * 2.0  # Scale factor
            
            # Angular velocity based on orientation (yaw)
            # Convert quaternion to yaw angle
            yaw = self.quaternion_to_yaw(feedback.pose.orientation)
            twist.angular.z = yaw * 2.0  # Scale factor

        # When the user releases the marker, stop the robot and reset the pose
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.reset_marker_pose(feedback.marker_name)

        self.cmd_pub.publish(twist)
        # self.get_logger().info(f'Published: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (rotation around z-axis)"""
        # Extract yaw from quaternion
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def reset_marker_pose(self, marker_name: str):
        neutral_pose = Pose()
        neutral_pose.orientation.w = 1.0
        self.server.setPose(marker_name, neutral_pose)
        self.server.applyChanges()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InteractiveMarkerTwist()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
