#!/usr/bin/python3

import rclpy
from rclpy.node import Node

# Import Message Types
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

# Import Libraries
from sensor_msgs_py import point_cloud2
import numpy as np
import tf_transformations

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')

        # Create Publisher & Timer to publish
        self.pub_pointcloud = self.create_publisher(PointCloud2, "map", 10)
        self.create_timer(0.1, self.timer_callback)  # Call the timer every 0.1 seconds

        # Create Subscribers
        self.create_subscription(PointCloud2, "/unilidar/cloud", self.lidar_callback, 10)

        # Class Variables
        self.points_array = None
        self.global_map = np.empty((0, 4), dtype=np.float32)  # Store [x, y, z, intensity]
        self.pointcloud_fields = None
        self.current_pose = None

    # Timer Callback ===================================================
    def timer_callback(self):
        # Publish the global map if we have accumulated points
        # if self.global_map is not None and self.pointcloud_fields is not None:
            # self.publish_pointcloud(self.global_map)

        if self.points_array is not None:
            self.publish_pointcloud(self.points_array)

    # Subscriber Callback for LiDAR data ================================
    def lidar_callback(self, msg):
        # Extract the fields dynamically from the sensor data (offsets for x, y, z, intensity)
        field_map = {field.name: field for field in msg.fields}
        self.pointcloud_fields = [
            field_map['x'],  # The field for 'x' with its offset
            field_map['y'],  # The field for 'y'
            field_map['z'],  # The field for 'z'
            field_map['intensity']  # The field for 'intensity'
        ]

        # Convert PointCloud2 message to structured NumPy array
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        self.points_array = np.array([list(p) for p in points], dtype=np.float32)


        # points_array = self.transform_to_global(points_array)

        # Stack the transformed points into the global map
        # self.global_map = np.vstack((self.global_map, points_array))



    # # Helper to transform points to the global frame ====================
    # def transform_to_global(self, points_array):
    #     if self.current_pose is None:
    #         self.get_logger().warn("No current pose available, cannot transform point cloud.")
    #         return points_array  # Return the points in their original frame

    #     # Extract position and orientation from the pose
    #     position = self.current_pose.position
    #     orientation = self.current_pose.orientation

    #     # Convert quaternion to a rotation matrix
    #     quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    #     rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]

    #     # Apply the transformation to the point cloud
    #     translation = np.array([position.x, position.y, position.z])
    #     points_xyz = points_array[:, :3]  # Extract [x, y, z]
    #     transformed_points = np.dot(points_xyz, rotation_matrix.T) + translation

    #     # Append the intensity to the transformed points
    #     global_points = np.hstack((transformed_points, points_array[:, 3].reshape(-1, 1)))

    #     return global_points

    # Publish point cloud data ==========================================
    def publish_pointcloud(self, points_array: np.ndarray):
        
        if points_array.size == 0:
            self.get_logger().warn("No points to publish.")
            return

        # Create the header for PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # Use current time
        header.frame_id = "map"

        # Create PointCloud2 message
        pc2_msg = point_cloud2.create_cloud(header, self.pointcloud_fields, points_array)
        self.pub_pointcloud.publish(pc2_msg)
        self.get_logger().info(f'Published point cloud with {len(points_array)} points')

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
