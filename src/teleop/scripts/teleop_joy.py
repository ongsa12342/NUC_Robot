#!/usr/bin/python3

from lecture2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyControlNode(Node):
    def __init__(self):
        super().__init__('joy_control_node')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/joy/cmd_vel', 10)

        # Initialize velocity variables
        self.linear_velocity = 1.0
        self.angular_velocity = 1.0
        self.linear_increment = 0.1  # Increment for linear velocity
        self.angular_increment = 0.1  # Increment for angular velocity

    def joy_callback(self, msg):
        twist = Twist()

        # Existing axes mappings
        # Example: Left stick for linear.x and angular.z (axes 1 and 0)
        twist.linear.x = msg.axes[1]  # Forward/backward (left stick vertical)
        twist.angular.z = msg.axes[0]  # Rotation (left stick horizontal)

        # Additional axes for new control
        # Right stick: axes 3 and 4 control linear y and x velocities
        twist.linear.y = msg.axes[3]  # Left/right control (right stick horizontal)
        twist.linear.x += msg.axes[4]  # Forward/backward (right stick vertical, added to left stick for combined control)

        # Buttons for adjusting velocity multipliers
        if msg.buttons[2]:  # X button pressed
            self.linear_velocity += self.linear_increment
            self.get_logger().info(f'Increased linear velocity: {self.linear_velocity}')
        elif msg.buttons[3]:  # Y button pressed
            self.linear_velocity -= self.linear_increment
            self.get_logger().info(f'Decreased linear velocity: {self.linear_velocity}')

        if msg.buttons[0]:  # A button pressed
            self.angular_velocity += self.angular_increment
            self.get_logger().info(f'Increased angular velocity: {self.angular_velocity}')
        elif msg.buttons[1]:  # B button pressed
            self.angular_velocity -= self.angular_increment
            self.get_logger().info(f'Decreased angular velocity: {self.angular_velocity}')

        # Apply the updated velocities
        twist.linear.x *= self.linear_velocity
        twist.linear.y *= self.linear_velocity
        twist.angular.z *= self.angular_velocity

        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
