#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import time
import math

class VelocityToOdometryConverter(Node):
    def __init__(self):
        super().__init__('velocity_to_odometry_converter')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,  
            '/my_response_topic',
            self.velocity_callback,
            10
        )
        self.odom_publisher = self.create_publisher(TransformStamped, '/wheel_odom', 10)
        self.timer = self.create_timer(0.1, self.update_odometry)  # 10 Hz
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_time = self.get_clock().now()

    def velocity_callback(self, msg):
        self.vx = msg.data[0]  # Linear velocity x
        self.vy = msg.data[1]  # Linear velocity y
        self.omega = msg.data[2]  # Angular velocity around z-axis

    def update_odometry(self):

        # Time update
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Update pose
        self.x += (self.vx * dt) * math.cos(self.theta) - (self.vy * dt) * math.sin(self.theta)
        self.y += (self.vx * dt) * math.sin(self.theta) + (self.vy * dt) * math.cos(self.theta)
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Create TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # Publish
        self.odom_publisher.publish(transform)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityToOdometryConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
