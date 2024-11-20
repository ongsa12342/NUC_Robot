import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ekf_launch_path = os.path.join(
        get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py'
    )

    node1 = Node(
        package='convert_odom',
        executable='accel_filter.py',
        name='accel_filter',
        output='screen',
    )

    node2 = Node(
        package='convert_odom',
        executable='convert_array_odom.py',
        name='convert_array_odom',
        output='screen',
    )
    
    node3 = Node(
        package='convert_odom',
        executable='drw_path.py',
        name='draw_path',
        output='screen',
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path)
    )
    
    return LaunchDescription([ekf_launch, node1, node2, node3])
