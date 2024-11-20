#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class VelocityToOdometryConverter(Node):
    def __init__(self):
        super().__init__('velocity_to_odometry_converter')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,  
            '/my_response_topic',
            self.velocity_callback,
            10
        )

        self.odom_publisher = self.create_publisher(Odometry, '/converted_odom', 10)

    def velocity_callback(self, msg):
        vx = msg.data[0]  # Linear velocity x
        vy = msg.data[1]  # Linear velocity y
        omega = msg.data[2]  # Angular velocity around z-axis

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = omega

    #     # odom_msg.twist.covariance = [
    #     #     0.1, 0,   0,   0,   0,   0,
    #     #     0,   0.1, 0,   0,   0,   0,
    #     #     0,   0,   0.1, 0,   0,   0,
    #     #     0,   0,   0,   0.1, 0,   0,
    #     #     0,   0,   0,   0,   0.1, 0,
    #     #     0,   0,   0,   0,   0,   0.1
    #     # ]

        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityToOdometryConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
