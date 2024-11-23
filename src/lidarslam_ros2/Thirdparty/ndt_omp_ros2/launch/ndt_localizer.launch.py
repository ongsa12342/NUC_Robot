from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # NDT-based localization node
        Node(
            package='ndt_omp',
            executable='ndt_localizer',
            output='screen',
            parameters=[
                {'global_frame_id': 'map'},
                {'robot_frame_id': 'base_link'},
                {'input_frame_id': 'unilidar_lidar'},
                {'pcd_map_file': '~/NUC_Robot/map.pcd'},
                {'use_imu': False},
                {'use_odom': True},
            ]
        ),
    ])
