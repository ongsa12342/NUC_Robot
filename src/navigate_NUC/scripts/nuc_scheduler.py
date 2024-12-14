#!/usr/bin/python3

import os
import yaml
from geometry_msgs.msg import PoseStamped
from navigate_NUC.dummy_module import dummy_function, dummy_var
from rclpy.node import Node
import rclpy
from std_srvs.srv import SetBool
from nuc_interfaces.srv import SetPoseStamped  # Assuming a custom Pose service is used

from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator



# State define here

# AUTONOMOUS
WAITING_FOR_NAVIGATE = 1
NAVIGATING = 2

# MANUAL
TELEOP = 4


class NUC_Scheduler(Node):
    def __init__(self):
        
        super().__init__('nuc_scheduler')

        # Parameters
        self.navigator = BasicNavigator()

        self.goal_pose = PoseStamped()

        # Services
        self.create_service(SetBool, 'is_nuc_auto', self.handle_is_nuc_auto)
        self.create_service(SetPoseStamped, 'nuc_goal_pose', self.handle_nuc_goal_pose)
        
        self.timer = self.create_timer(100, self.timer_callback) #100 hz

        self.state = TELEOP #init
        
        self.get_logger().info('NUC Scheduler Node is up and running.')

    def handle_is_nuc_auto(self, request, response):
        """Service handler for is_nuc_auto."""
        is_auto_mode = request.data
        CMD_VEL = 0 # waut for joy
        if is_auto_mode:
            if self.state == TELEOP:
                if CMD_VEL == 0:
                    response.success = False
                    self.get_logger().info("Please wait until robot stop!!!")
                else:
                    response.success = True
                    self.state = WAITING_FOR_NAVIGATE
                    self.get_logger().info("Changed to Autonomous mode")
                    
            elif self.state == WAITING_FOR_NAVIGATE:
                
                response.success = True
                self.get_logger().info("Now is already Autonomous mode")
            else:
                response.success = False
                self.get_logger().info("Can not change to Autonomous mode")
        else:
            if self.state == WAITING_FOR_NAVIGATE:
                self.state = TELEOP
                response.success = True
                self.get_logger().info("Changed to Manual mode")
            elif self.state == TELEOP:
                
                response.success = True
                self.get_logger().info("Now is already Manual mode")
                
            else:
                response.success = False
                self.get_logger().info("Can not change to Manual mode")
        
        response.message = f"Auto mode set to {is_auto_mode}."
        self.get_logger().info(response.message)
        self.get_logger().info(f"Current state: {self.state}")
        
        return response

    def handle_nuc_goal_pose(self, request, response):
        """Service handler for nuc_goal_pose."""

        if self.state == WAITING_FOR_NAVIGATE:
            # Manually assign position fields
            self.goal_pose.pose.position.x = request.pose.position.x
            self.goal_pose.pose.position.y = request.pose.position.y
            self.goal_pose.pose.position.z = request.pose.position.z

            # Manually assign orientation fields
            self.goal_pose.pose.orientation.x = request.pose.orientation.x
            self.goal_pose.pose.orientation.y = request.pose.orientation.y
            self.goal_pose.pose.orientation.z = request.pose.orientation.z
            self.goal_pose.pose.orientation.w = request.pose.orientation.w

            # Add header information
            self.goal_pose.header.frame_id = 'map'  # Set frame ID to 'map'
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()  # Add current timestamp

            # Log the goal pose details
            self.get_logger().info(
                f"Goal pose set manually: Frame = {self.goal_pose.header.frame_id}, "
                f"Position = ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y}, {self.goal_pose.pose.position.z}), "
                f"Orientation = ({self.goal_pose.pose.orientation.x}, {self.goal_pose.pose.orientation.y}, "
                f"{self.goal_pose.pose.orientation.z}, {self.goal_pose.pose.orientation.w})"
            )
            
            #sent to nav goal here
            
            self.get_logger().info("dummy sent pose here")
            # self.navigator.goToPose(goal_pose)
            
            response.success = True
            self.state = NAVIGATING
            
        elif self.state == NAVIGATING:
            response.success = False
            self.get_logger().info("Please wait!! Navigating")
        else:
            response.success = False
            self.get_logger().info("Can not Navigate in this mode")

        
        return response
    def FSM(self):
        if self.state == NAVIGATING:
            # result = self.navigator.getResult()
            result = True

            if result:
                self.get_logger().info('Goal Successed')
                self.state = WAITING_FOR_NAVIGATE
        
            
            

        
    def timer_callback(self):
        self.FSM()
        
def main(args=None):
    rclpy.init(args=args)
    node = NUC_Scheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NUC Scheduler Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
