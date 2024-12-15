#!/usr/bin/python3

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QRadioButton, QLabel, QWidget, QAction, QMessageBox, QInputDialog, QMenu
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QCursor  # or PySide2.QtGui, depending on your framework

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from std_srvs.srv import SetBool
from nuc_interfaces.srv import SetPoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtWidgets import QSpacerItem, QSizePolicy
from PyQt5.QtCore import QTimer

TRUE = SetBool.Request()
TRUE.data = True

FALSE = SetBool.Request()
FALSE.data = False

MODE_TELEOP = 0
MODE_AUTO = 1

class NucUserInterface(Node):
    def __init__(self):
        super().__init__('nuc_user_interface')

        # Service clients
        self.is_nuc_auto = self.create_client(SetBool, 'is_nuc_auto')
        self.nuc_goal_pose = self.create_client(SetPoseStamped, '/nuc_goal_pose')

        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Locate the YAML file in the 'param' directory of the package
        self.yaml_file = os.path.join(
            os.getcwd(),  # Current working directory
            'src/navigate_NUC',  # Path to the package source
            'params',
            'room_coordinates.yaml'
        )

        print(self.yaml_file)
        self.room_coordinates = self.load_coordinates()
        self.home_data = self.get_home_data()
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10  # QoS (Queue size)
        )

        self.current_amcl_pose = None
    def amcl_pose_callback(self, msg):
        self.current_amcl_pose = msg

    def save_room(self, room_name):
        if not self.current_amcl_pose:
            QMessageBox.warning(None, "AMCL Pose Not Available", "Current AMCL pose is not available. Cannot save room.")
            return

        position = {
            'x': self.current_amcl_pose.pose.pose.position.x,
            'y': self.current_amcl_pose.pose.pose.position.y
        }
        orientation = {
            'z': self.current_amcl_pose.pose.pose.orientation.z,
            'w': self.current_amcl_pose.pose.pose.orientation.w
        }

        self.room_coordinates.append({
            'name': room_name,
            'position': position,
            'orientation': orientation
        })

        self.save_coordinates_to_yaml()
        self.get_logger().info(f"Room '{room_name}' saved: position={position}, orientation={orientation}")

    def save_coordinates_to_yaml(self):
        with open(self.yaml_file, 'w') as file:
            yaml.dump({'rooms': self.room_coordinates}, file)
        self.get_logger().info(f"Saved coordinates to YAML: {self.yaml_file}\n{self.room_coordinates}")


    def set_mode(self, auto_mode):
        request = TRUE if auto_mode else FALSE
        if self.is_nuc_auto.wait_for_service(timeout_sec=1.0):
            future = self.is_nuc_auto.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f"Mode set: {'Auto' if auto_mode else 'Manual'}")
                    return True
                else:
                    self.get_logger().error("Service call failed: " + future.result().message)
                    return False
            else:
                self.get_logger().error("Failed to call service.")
                return False
        else:
            self.get_logger().error("Service 'is_nuc_auto' not available.")
            return False

    def pose_estimate(self, position, orientation):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = position['x']
        pose_msg.pose.pose.position.y = position['y']
        pose_msg.pose.pose.position.z = 0.0

        # Normalize quaternion
        magnitude = (orientation['z']**2 + orientation['w']**2)**0.5
        pose_msg.pose.pose.orientation.z = orientation['z'] / magnitude
        pose_msg.pose.pose.orientation.w = orientation['w'] / magnitude

        # Covariance matrix
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"Published initial pose: position={position}, orientation={orientation}")


    def send_goal_pose(self, position, orientation):
        request = SetPoseStamped.Request()
        request.pose.position.x = position['x']
        request.pose.position.y = position['y']
        request.pose.position.z = 0.0
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = orientation.get('z', 0.0)
        request.pose.orientation.w = orientation['w']

        if self.nuc_goal_pose.wait_for_service(timeout_sec=1.0):
            future = self.nuc_goal_pose.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f"Goal pose sent successfully: position={position}, orientation={orientation}")
                else:
                    self.get_logger().error("Failed to send goal pose: " + future.result().message)
            else:
                self.get_logger().error("Failed to call goal pose service.")
        else:
            self.get_logger().error("Service '/nuc_goal_pose' not available.")

    def load_coordinates(self):
        try:
            with open(self.yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                for room in data['rooms']:
                    if 'position' not in room or 'orientation' not in room:
                        self.get_logger().warning(f"Room {room.get('name', 'Unknown')} is missing position or orientation data.")
                return data['rooms']
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found: {self.yaml_file}")
            return []

    def get_home_data(self):
        for room in self.room_coordinates:
            if room["name"] == 'home':
                return {
                    'position': room['position'],
                    'orientation': room['orientation']
                }
        self.get_logger().error("Home data not found in YAML.")
        return None

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("NUC User Interface")

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QVBoxLayout()

        # Home button
        self.home_button = QPushButton("Set Home")
        self.home_button.clicked.connect(self.set_home)
        self.layout.addWidget(self.home_button)

        # Save Room button
        self.save_room_button = QPushButton("Save Current Room")
        self.save_room_button.clicked.connect(self.save_room)
        self.layout.addWidget(self.save_room_button)

        # Mode toggle buttons
        self.layout.addWidget(QLabel("Mode Selection:", alignment=Qt.AlignCenter))

        self.manual_button = QRadioButton("Manual")
        self.manual_button.setChecked(True)
        self.manual_button.toggled.connect(self.toggle_mode)
        self.layout.addWidget(self.manual_button)

        self.auto_button = QRadioButton("Auto")
        self.layout.addWidget(self.auto_button)

        central_widget.setLayout(self.layout)

        # Room buttons (added dynamically in Auto mode)
        self.room_buttons = []
        self.create_room_buttons()
        self.update_buttons_visibility()

        spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.layout.addItem(spacer)

    def create_room_buttons(self):
        for idx, room in enumerate(self.ros_node.room_coordinates):
            button = QPushButton(f"Room: {room.get('name', 'Unknown')}")
            button.setContextMenuPolicy(Qt.CustomContextMenu)
            button.customContextMenuRequested.connect(lambda pos, b=button, r=room: self.show_context_menu(b, r))
            button.clicked.connect(lambda _, r=room: self.select_room(r))
            self.room_buttons.append(button)
            self.layout.addWidget(button)

    def show_context_menu(self, button, room):
        menu = QMenu(self)
        delete_action = QAction("Delete Room", self)
        delete_action.triggered.connect(lambda: self.delete_room(button, room))
        menu.addAction(delete_action)
        
        # Properly use QCursor for the global position
        global_pos = button.mapToGlobal(button.rect().center())
        menu.exec_(global_pos)



    def delete_room(self, button, room):
        self.ros_node.room_coordinates.remove(room)
        self.ros_node.save_coordinates_to_yaml()
        button.setParent(None)
        self.room_buttons.remove(button)
        QMessageBox.information(self, "Room Deleted", f"Room '{room['name']}' has been deleted.")

    def save_room(self):
        room_name, ok = QInputDialog.getText(self, "Save Room", "Enter room name:")
        if ok and room_name:
            self.ros_node.save_room(room_name)
            QMessageBox.information(self, "Room Saved", f"Room '{room_name}' has been saved.")

    def update_buttons_visibility(self):
        is_auto = self.auto_button.isChecked()
        for button in self.room_buttons:
            button.setVisible(is_auto)
        self.home_button.setVisible(True) 
        self.save_room_button.setVisible(not is_auto)
        # Update layout to adjust dynamically
        self.layout.update()


    def select_room(self, room):
        if 'position' not in room or 'orientation' not in room:
            QMessageBox.warning(self, "Room Selection Failed", f"Cannot select room: {room.get('name', 'Unknown')}. Missing position or orientation.")
            return

        position = room['position']
        orientation = room['orientation']
        self.ros_node.get_logger().info(f"Selected Room: {room.get('name', 'Unknown')} (position: {position}, orientation: {orientation})")
        self.ros_node.send_goal_pose(position, orientation)

    def set_home(self):
        if not self.ros_node.home_data:
            QMessageBox.critical(self, "Home Data Missing", "Home data is not available in the YAML file.")
            return

        position = self.ros_node.home_data['position']
        orientation = self.ros_node.home_data['orientation']
        self.ros_node.get_logger().info(f"Setting Home: position={position}, orientation={orientation}")
        self.ros_node.pose_estimate(position, orientation)

    def toggle_mode(self):
        is_auto = self.auto_button.isChecked()
        # Prevent re-triggering toggle if mode change fails
        self.manual_button.blockSignals(True)
        self.auto_button.blockSignals(True)

        if not self.ros_node.set_mode(is_auto):
            QMessageBox.critical(self, "Mode Change Failed", "Unable to change mode. Please try again.")
            # Revert the toggle state
            if is_auto:
                self.manual_button.setChecked(True)
            else:
                self.auto_button.setChecked(True)
        else:
            self.update_buttons_visibility()

        self.manual_button.blockSignals(False)
        self.auto_button.blockSignals(False)

def main():
    rclpy.init()
    ros_node = NucUserInterface()

    app = QApplication(sys.argv)
    main_window = MainWindow(ros_node)
    main_window.show()

    # Create a QTimer to periodically spin the ROS 2 node
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.1))
    timer.start(10)  # Spin the ROS 2 node every 10 ms

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

