import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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

    nuc_scheduler_action = ExecuteProcess(
        cmd=['ros2', 'run', 'navigate_NUC', 'nuc_scheduler.py'],
        output='screen'
    )

    ekf_launch_path = os.path.join(
        get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py'
    )
    point_to_laser_launch_path = os.path.join(
        get_package_share_directory('convert_odom'), 'launch', 'custom_pointcloud_to_laserscan_launch.py'
    )

    Filter_LP_node = Node(
        package='convert_odom',
        executable='accel_filter.py',
        name='accel_filter',
        output='screen',
    )

    convert_odom_type_node = Node(
        package='convert_odom',
        executable='convert_array_odom.py',
        name='convert_array_odom',
        output='screen',
    )

    time_sync_node = Node(
        package='convert_odom',
        executable='time_sync.py',
        name='Synctime',
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

    point_to_laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(point_to_laser_launch_path)
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path)
    )

    ld.add_action(unitree_lidar_action)
    ld.add_action(uros_action)
    ld.add_action(teleop_joy_action)
    ld.add_action(teleop_twist_joy_action)
    ld.add_action(nuc_scheduler_action)
    ld.add_action(ekf_launch)
    ld.add_action(Filter_LP_node)
    ld.add_action(convert_odom_type_node)
    ld.add_action(time_sync_node)
    ld.add_action(tf)
    ld.add_action(point_to_laser_launch)

    return ld
