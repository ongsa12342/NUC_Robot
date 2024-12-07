import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("convert_odom"),
                                   'config', 'slam_config.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    simple_launch_path = os.path.join(
        get_package_share_directory('convert_odom'), 'launch', 'simple.launch.py'
    )
    point_to_laser_launch_path = os.path.join(
        get_package_share_directory('convert_odom'), 'launch', 'custom_pointcloud_to_laserscan_launch.py'
    )

    convert_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simple_launch_path)
    )

    point_to_laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(point_to_laser_launch_path)
    )

    # Add RViz2 node
    rviz_config_path = os.path.join(
        get_package_share_directory('convert_odom'), 'config', 'Slam_mapping.rviz'
    )

    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(convert_launch)
    # ld.add_action(point_to_laser_launch)
    ld.add_action(start_rviz_node)

    return ld
