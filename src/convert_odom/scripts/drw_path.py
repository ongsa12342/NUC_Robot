#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')
        
        # Subscribe to the /odometry/filtered topic
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Publish the path to /robot_path
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        
        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "odom"  # Update frame_id to match your TF frame

    def odom_callback(self, msg):
        """Callback for processing Odometry messages."""
        # Create a PoseStamped message from the Odometry data
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Append the pose to the Path
        self.path.poses.append(pose_stamped)

        # Update the Path header timestamp
        self.path.header.stamp = self.get_clock().now().to_msg()

        # Publish the updated Path
        self.path_pub.publish(self.path)

        # Log the position for debugging
        self.get_logger().info(f"Published Path: x={pose_stamped.pose.position.x:.3f}, y={pose_stamped.pose.position.y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
