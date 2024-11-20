import os
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # Run convert
  node1 = Node(
    package='convert_odom',
    executable='accel_filter.py',
    name='Accfil',
  )
  node2 = Node(
    package='convert_odom',
    executable='convert_array_odom.py',
    name='Accfil',
)

  # # Run Rviz
  # package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
  # rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
  # print("rviz_config_file = " + rviz_config_file)
  # rviz_node = Node(
  #   package='rviz2',
  #   executable='rviz2',
  #   name='rviz2',
  #   arguments=['-d', rviz_config_file],
  #   output='log'
  # )
  # return LaunchDescription([node1, rviz_node])

  return LaunchDescription([node1,node2])