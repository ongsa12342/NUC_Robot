#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class TimestampFixerNode(Node):
    def __init__(self):
        super().__init__('timestamp_fixer_node')

        # Subscribe to the PointCloud2 topic from ros2 bag
        self.subscription = self.create_subscription(
            PointCloud2,
            'unilidar/cloud',  # Original PointCloud2 topic from the bag
            self.pointcloud_callback,
            10
        )

        # Publisher for the updated PointCloud2
        self.publisher = self.create_publisher(
            PointCloud2,
            'unilidar/cloud_fixed',  # New topic with fixed timestamps
            10
        )

    def pointcloud_callback(self, msg):
        # Update the timestamp to the current simulation time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the updated message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimestampFixerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
