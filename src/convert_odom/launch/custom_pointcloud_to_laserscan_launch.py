from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare the namespace for remapping
        DeclareLaunchArgument(
            name='scanner', default_value='unilidar',
            description='Namespace for the topics'
        ),

        # Static Transform Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '0', '0', '0',  # x, y, z
                '0', '0', '-0.195', '0.981',  # Quaternion qx, qy, qz, qw
                'base_link',  # Parent frame
                'unilidar_lidar'  # Child frame
            ]
        ),

        # PointCloud2 to LaserScan conversion
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/cloud_fixed']),
                # ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])
            ],
            parameters=[{
                'target_frame': 'unilidar_lidar',  # Frame in which the scan will be published
                'transform_tolerance': 0.1,
                'queue_size': 10,  # Increase from the default
                'min_height': -2.0,  # Height limits to filter the PointCloud
                'max_height': 2.0,
                'angle_min': -3.14159,  # Full 360-degree scan (-pi)
                'angle_max': 3.14159,  # Full 360-degree scan (+pi)
                'angle_increment': 0.0087,  # Angle resolution (rad)
                'scan_time': 0.1,
                'range_min': 0.1,  # Minimum valid range
                'range_max': 40.0,  # Maximum valid range
                'use_inf': True,  # Use inf values for out-of-range
                'inf_epsilon': 1.0,
                'use_sim_time': False
            }],
            name='pointcloud_to_laserscan'
        )
    ])
