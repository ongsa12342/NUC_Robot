#!/usr/bin/python3

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QRadioButton, QLabel, QWidget, QButtonGroup, QMessageBox
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from std_srvs.srv import SetBool
from nuc_interfaces.srv import SetPoseStamped

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

        # Locate the YAML file in the 'param' directory of the package
        yaml_file = os.path.join(
            get_package_share_directory('navigate_NUC'),  # Replace with your package name
            'params',
            'room_coordinates.yaml'  # YAML file name
        )
        self.room_coordinates = self.load_coordinates(yaml_file)

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

    def send_goal_pose(self, x, y, orientation_w=1.0):
        request = SetPoseStamped.Request()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = 0.0
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0
        request.pose.orientation.w = orientation_w

        if self.nuc_goal_pose.wait_for_service(timeout_sec=1.0):
            future = self.nuc_goal_pose.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f"Goal pose sent successfully: x={x}, y={y}, orientation_w={orientation_w}")
                else:
                    self.get_logger().error("Failed to send goal pose: " + future.result().message)
            else:
                self.get_logger().error("Failed to call goal pose service.")
        else:
            self.get_logger().error("Service '/nuc_goal_pose' not available.")

    def load_coordinates(self, yaml_file):
        try:
            with open(yaml_file, 'r') as file:
                data = yaml.safe_load(file)
                for room in data['rooms']:
                    if 'orientation_w' not in room:
                        room['orientation_w'] = 1.0
                return data['rooms']
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found: {yaml_file}")
            return []

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("NUC User Interface")

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()

        # Room selection label
        layout.addWidget(QLabel("Select a Room:", alignment=Qt.AlignCenter))

        # Room buttons
        self.room_group = QButtonGroup()
        for idx, room in enumerate(self.ros_node.room_coordinates):
            button = QPushButton(f"Room {idx + 1}: {room['name']}")
            button.clicked.connect(lambda _, r=room: self.select_room(r))
            self.room_group.addButton(button)
            layout.addWidget(button)

        # Mode toggle buttons
        layout.addWidget(QLabel("Mode Selection:", alignment=Qt.AlignCenter))

        self.manual_button = QRadioButton("Manual")
        self.manual_button.setChecked(True)
        self.manual_button.toggled.connect(self.toggle_mode)
        layout.addWidget(self.manual_button)

        self.auto_button = QRadioButton("Auto")
        layout.addWidget(self.auto_button)

        central_widget.setLayout(layout)

    def select_room(self, room):
        self.ros_node.get_logger().info(f"Selected Room: {room['name']} (x: {room['x']}, y: {room['y']}, orientation_w: {room['orientation_w']})")
        self.ros_node.send_goal_pose(room['x'], room['y'], room['orientation_w'])

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
        self.manual_button.blockSignals(False)
        self.auto_button.blockSignals(False)

def main():
    rclpy.init()
    ros_node = NucUserInterface()

    app = QApplication(sys.argv)
    main_window = MainWindow(ros_node)
    main_window.show()

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
