#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class ImuCovarianceNode(Node):
    def __init__(self):
        super().__init__('imu_covariance_node')

        # Declare and get parameters
        self.declare_parameter('imu_topic', '/unilidar/imu')  # Updated IMU topic
        self.declare_parameter('output_topic', '/imu/covariance')
        self.declare_parameter('buffer_size', 10000)
        self.declare_parameter('update_rate', 220.0)  # Hz

        self.imu_topic = self.get_parameter('imu_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.update_rate = self.get_parameter('update_rate').value

        # Buffers for IMU data
        self.angular_velocity_data = []
        self.linear_acceleration_data = []

        # Subscription
        self.subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )
        self.get_logger().info(f"Subscribed to IMU topic: {self.imu_topic}")

        # Publisher
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        self.get_logger().info(f"Publishing covariance to: {self.output_topic}")

        # Timer for periodic computation
        self.timer = self.create_timer(1.0 / self.update_rate, self.compute_and_publish_covariance)

    def imu_callback(self, msg: Imu):
        # Append angular velocity and linear acceleration to buffers
        self.angular_velocity_data.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.linear_acceleration_data.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # Maintain buffer size
        if len(self.angular_velocity_data) > self.buffer_size:
            self.angular_velocity_data.pop(0)
        if len(self.linear_acceleration_data) > self.buffer_size:
            self.linear_acceleration_data.pop(0)

    def compute_and_publish_covariance(self):
        if len(self.angular_velocity_data) < 2:
            self.get_logger().info("Not enough data for covariance calculation.")
            return

        # Convert buffers to NumPy arrays
        angular_velocity_array = np.array(self.angular_velocity_data)
        linear_acceleration_array = np.array(self.linear_acceleration_data)

        # Calculate covariance matrices
        angular_velocity_cov = np.cov(angular_velocity_array, rowvar=False)
        linear_acceleration_cov = np.cov(linear_acceleration_array, rowvar=False)

        # Log the covariance matrices (for debugging)
        self.get_logger().debug("Angular Velocity Covariance Matrix:\n" + str(angular_velocity_cov))
        self.get_logger().debug("Linear Acceleration Covariance Matrix:\n" + str(linear_acceleration_cov))

        # Create and publish PoseWithCovarianceStamped message
        covariance_msg = PoseWithCovarianceStamped()
        covariance_msg.header.stamp = self.get_clock().now().to_msg()
        covariance_msg.header.frame_id = "imu"

        # Fill in covariance (flattened 6x6 matrix)
        # Assuming angular velocity and linear acceleration covariance
        combined_covariance = np.zeros((6, 6))
        combined_covariance[0:3, 0:3] = angular_velocity_cov  # Angular velocity
        combined_covariance[3:6, 3:6] = linear_acceleration_cov  # Linear acceleration
        covariance_msg.pose.covariance = combined_covariance.flatten().tolist()

        # Publish message
        self.publisher.publish(covariance_msg)
        self.get_logger().info("Published IMU covariance data.")

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
