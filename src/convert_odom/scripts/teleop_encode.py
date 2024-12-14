#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from std_srvs.srv import SetBool



class TeleEncoderNode(Node):
    def __init__(self):
        super().__init__('teleop_encode')
        # self.create_subscription(Twist, '/robot/cmd_vel',self.cmdvel_callback,10)
        self.pub_vel = self.create_publisher(Float32MultiArray,'/float_dumb_topic',10)
        self.rate = 100.0
        # self.create_timer(1/self.rate,self.timer_callback)
        self.velo = np.array([0.0,0.0,0.0])
        
        self.create_service(SetBool, 'is_cmd_auto', self.handle_is_cmd_auto)
        self.is_cmd_auto = False
                # sub
        self.create_subscription(
            Twist,  
            '/joy/cmd_vel',
            self.handle_joy_callback,
            10
        )
        self.create_subscription(
            Twist,  
            '/cmd_vel',
            self.handle_nav_callback,
            10
        )
           
    def handle_is_cmd_auto(self, request, response):
        """Service handler for is_nuc_auto."""
        is_cmd_auto = request.data
        self.is_cmd_auto = is_cmd_auto
        return response

    def handle_joy_callback(self,msg):
        if not self.is_cmd_auto:
            velo = [msg.linear.x, msg.linear.y, msg.angular.z]

            pub = Float32MultiArray()
            pub.data = velo
            self.pub_vel.publish(pub)

    def handle_nav_callback(self,msg):
        if self.is_cmd_auto:
            velo = [msg.linear.x, msg.linear.y, msg.angular.z]

            pub = Float32MultiArray()
            pub.data = velo
            self.pub_vel.publish(pub)
        
    # def cmdvel_callback(self,msg:Twist):
    #     self.velo[0] = msg.linear.x
    #     self.velo[1] = msg.linear.y
    #     self.velo[2] = msg.angular.z
    # def timer_callback(self):
    #     # pass
    #     msg = Float32MultiArray()
    #     msg.data = self.velo.tolist()
    #     self.pub_vel.publish(msg)
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TeleEncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
