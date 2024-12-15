#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory
import yaml
import os

class NavigationGoalSender(Node):
    def __init__(self):
        super().__init__('navigation_goal_sender')
        # Locate the YAML file in the 'param' directory of the package
        yaml_file = os.path.join(
            get_package_share_directory('navigate_NUC'),  # Replace with your package name
            'params',
            'room_coordinates.yaml'  # YAML file name
        )
        self.room_coordinates = self.load_coordinates(yaml_file)
        self.navigator = BasicNavigator()

    def load_coordinates(self, yaml_file):
        """
        Load room coordinates from a YAML file.

        Args:
            yaml_file (str): Path to the YAML file.

        Returns:
            list: List of room dictionaries.
        """
        try:
            with open(yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                return data['rooms']
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found: {yaml_file}")
            return []

    def send_goal(self, room):
        """
        Send a navigation goal to a specific room and allow cancellation.

        Args:
            room (dict): Room dictionary containing name, x, y, and orientation.
        """
        self.get_logger().info("Waiting for Nav2 system...")
        self.navigator.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = room['x']
        goal_pose.pose.position.y = room['y']
        goal_pose.pose.orientation.w = room.get('orientation_w', 1.0)

        self.get_logger().info(f"Sending goal to room {room['name']} at ({room['x']}, {room['y']})")
        self.navigator.goToPose(goal_pose)

        # Monitor the task and provide the option to cancel
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} meters")

            # Check for user input to cancel the task
            user_input = input("Press 'c' to cancel the goal, or any other key to continue: ").strip().lower()
            if user_input == 'c':
                self.navigator.cancelTask()
                self.get_logger().info(f"Goal for room {room['name']} was canceled by the user.")
                return  # Exit the method once the task is canceled

        # Get the result of the task
        result = self.navigator.getResult()

        if result == 0:  # Succeeded
            self.get_logger().info(f"Goal for room {room['name']} succeeded!")
        elif result == 1:  # Canceled
            self.get_logger().info(f"Goal for room {room['name']} was canceled.")
        else:  # Failed
            self.get_logger().info(f"Goal for room {room['name']} failed.")


def main():
    rclpy.init()
    goal_sender = NavigationGoalSender()

    try:
        while True:
            # Display available rooms
            print("\nAvailable rooms:")
            for idx, room in enumerate(goal_sender.room_coordinates):
                print(f"{idx + 1}. {room['name']}")

            # Prompt user for input
            choice = input("Enter the number of the room to navigate to (or 'q' to quit): ").strip()
            if choice.lower() == 'q':
                print("Exiting.")
                break

            try:
                room_idx = int(choice) - 1
                if 0 <= room_idx < len(goal_sender.room_coordinates):
                    goal_sender.send_goal(goal_sender.room_coordinates[room_idx])
                else:
                    print("Invalid selection. Please choose a valid room number.")
            except ValueError:
                print("Invalid input. Please enter a number corresponding to a room or 'q' to quit.")

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
