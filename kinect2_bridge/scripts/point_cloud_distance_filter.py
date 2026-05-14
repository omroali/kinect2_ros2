#!/usr/bin/env python3
"""
Simple point cloud distance filter node.
Filters PointCloud2 messages by Z-axis distance (depth from camera).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np


class PointCloudDistanceFilter(Node):
    def __init__(self):
        super().__init__("point_cloud_distance_filter")
        
        self.declare_parameter("min_distance", 0.0)
        self.declare_parameter("max_distance", 5.0)
        
        self.min_dist = self.get_parameter("min_distance").value
        self.max_dist = self.get_parameter("max_distance").value
        
        self.subscription = self.create_subscription(
            PointCloud2,
            "input",
            self.filter_callback,
            10,
        )
        self.publisher = self.create_publisher(PointCloud2, "output", 10)
        
        self.get_logger().info(
            f"Point cloud distance filter started: {self.min_dist:.2f}m to {self.max_dist:.2f}m"
        )
    
    def filter_callback(self, msg: PointCloud2):
        # Convert ROS message to numpy structured array
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
        
        # Parse point cloud structure: assumes RGB or standard XYZ layout
        # For Kinect: float32 X, Y, Z followed by RGB data
        point_step = msg.point_step
        row_step = msg.row_step
        width = msg.width
        height = msg.height
        
        # Extract Z values (depth) - assume Z is typically at byte offset 8 (after X, Y which are 4 bytes each)
        z_offset = 8
        z_values = np.frombuffer(data, dtype=np.float32, offset=z_offset, count=width * height)
        
        # Create mask: keep points within distance range
        mask = (z_values >= self.min_dist) & (z_values <= self.max_dist)
        filtered_indices = np.where(mask)[0]
        
        if len(filtered_indices) == 0:
            # No points in range, send empty cloud
            empty_msg = PointCloud2()
            empty_msg.header = msg.header
            empty_msg.width = 0
            empty_msg.height = 0
            empty_msg.fields = msg.fields
            empty_msg.is_bigendian = msg.is_bigendian
            empty_msg.point_step = msg.point_step
            empty_msg.row_step = 0
            empty_msg.data = b''
            empty_msg.is_dense = msg.is_dense
            self.publisher.publish(empty_msg)
            return
        
        # Build filtered data
        filtered_data = bytearray()
        for idx in filtered_indices:
            start = idx * point_step
            end = start + point_step
            filtered_data.extend(data[start:end])
        
        # Create filtered message
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = 1
        filtered_msg.width = len(filtered_indices)
        filtered_msg.fields = msg.fields
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = len(filtered_indices) * point_step
        filtered_msg.data = bytes(filtered_data)
        filtered_msg.is_dense = msg.is_dense
        
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
