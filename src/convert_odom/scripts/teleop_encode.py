#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np


class TeleEncoderNode(Node):
    def __init__(self):
        super().__init__('teleop_encode')
        self.create_subscription(Twist, '/cmd_vel',self.cmdvel_callback,10)
        self.pub_vel = self.create_publisher(Float32MultiArray,'/float_dumb_topic',10)
        self.rate = 100.0
        self.create_timer(1/self.rate,self.timer_callback)
        self.velo = np.array([0.0,0.0,0.0])
    def cmdvel_callback(self,msg:Twist):
        self.velo[0] = msg.linear.x
        self.velo[1] = msg.linear.y
        self.velo[2] = msg.angular.z
        # self.get_logger().info(f'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa {self.velo[0]}')
    def timer_callback(self):
        # pass
        msg = Float32MultiArray()
        msg.data = self.velo.tolist()
        # self.get_logger(f'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa {msg}')
        self.pub_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleEncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
