#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu
import numpy as np

class LowPassFilterNode(Node):
    def __init__(self):
        super().__init__('low_pass_filter_node')

        self.declare_parameter('alpha', 0.1)  
        self.alpha = self.get_parameter('alpha').value

        self.prev_filtered_accel = np.zeros(3)  
        self.prev_filtered_ang_vel_z = 0.0  # 

        self.subscription = self.create_subscription(Imu, '/unilidar/imu',self.imu_callback,20) 
        self.publisher_ = self.create_publisher(Imu, '/imu/filtered_LPS', 20)
        # Timer
        self.time = 0.02
        self.create_timer(self.time,self.timer_callback)
        # Degree 
        theta_deg = -22.5
        alpha_deg = 0
        theta = np.radians(theta_deg)  
        alpha = np.radians(alpha_deg)

        self.rotation_matrix_z = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0,0,1]
        ])
        
        self.rotation_matrix_x = np.array([
            [1,0,0],
            [0, np.cos(alpha), -np.sin(alpha)],
            [0, np.sin(alpha), np.cos(alpha)]
        ])
        self.filtered_msg = Imu()
        
    def imu_callback(self, msg):
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        ang_vel_z = msg.angular_velocity.z

        accel_rotated = np.dot(np.dot(self.rotation_matrix_z, self.rotation_matrix_x )  , accel)


        filtered_accel = self.alpha * accel_rotated + (1 - self.alpha) * self.prev_filtered_accel
        self.prev_filtered_accel = filtered_accel

        filtered_ang_vel_z = self.alpha * ang_vel_z + (1 - self.alpha) * self.prev_filtered_ang_vel_z
        self.prev_filtered_ang_vel_z = filtered_ang_vel_z

        # filtered_msg = Imu()
        self.filtered_msg.header = msg.header
        self.filtered_msg.orientation = msg.orientation

        self.filtered_msg.angular_velocity.x = msg.angular_velocity.x
        self.filtered_msg.angular_velocity.y = msg.angular_velocity.y
        self.filtered_msg.angular_velocity.z = filtered_ang_vel_z

        self.filtered_msg.linear_acceleration.x = filtered_accel[0]
        self.filtered_msg.linear_acceleration.y = filtered_accel[1]
        self.filtered_msg.linear_acceleration.z = msg.linear_acceleration.z

        self.filtered_msg.orientation_covariance = msg.orientation_covariance
        self.filtered_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        self.filtered_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
    def timer_callback(self):
        self.publisher_.publish(self.filtered_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = LowPassFilterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
