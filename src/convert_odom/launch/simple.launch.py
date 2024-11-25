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
    point_to_laser_launch_path = os.path.join(
        get_package_share_directory('convert_odom'), 'launch', 'custom_pointcloud_to_laserscan_launch.py'
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
        # parameters=[{'use_sim_time': True}],  # Enable sim time
    )
    node4 = Node(
    package='convert_odom',
    executable='time_sync.py',
    name='Synctime',
    )
    
    node3 = Node(
        package='convert_odom',
        executable='drw_path.py',
        name='draw_path',
        output='screen',
    )
    
    tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '0', '0', '0',  # x, y, z
                '0', '0', '0', '1',  # Quaternion qx, qy, qz, qw
                'map',  # Parent frame
                'odom'  # Child frame
            ]
    )
    
    robot_state = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publish',
           parameters=[
                {'robot_description': open('/home/farao/Desktop/urdf/nuc.urdf').read()}
            ],
    )
    
    point_to_laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(point_to_laser_launch_path)
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path)
    )
    
    return LaunchDescription([ekf_launch, node1, node2,node4,tf,point_to_laser_launch])