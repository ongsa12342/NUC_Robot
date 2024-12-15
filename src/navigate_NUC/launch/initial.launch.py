import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    unitree_lidar_action = ExecuteProcess(
        cmd=['ros2', 'launch', 'unitree_lidar_ros2', 'launch.py'],
        output='screen'
    )

    uros_action = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_serial',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '921600']
    )

    teleop_joy_action = Node(
        package='teleop',
        executable='teleop_joy.py',
        name='teleop_joy',
        output='screen'
    )

    teleop_twist_joy_action = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen'
    )

    ld.add_action(unitree_lidar_action)
    ld.add_action(uros_action)
    ld.add_action(teleop_joy_action)
    ld.add_action(teleop_twist_joy_action)

    return ld
