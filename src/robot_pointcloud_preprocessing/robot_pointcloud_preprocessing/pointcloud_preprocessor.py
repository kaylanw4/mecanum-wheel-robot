#!/usr/bin/env python3
"""
Point Cloud Preprocessing Node for ZED2i Camera Data
Phase 2: Statistical outlier removal with k-nearest neighbor filtering

This node subscribes to the raw ZED2i point cloud and republishes it as a cleaned version.
Phase 2 adds statistical outlier removal to filter spurious points from reflective surfaces.

File: src/robot_pointcloud_preprocessing/robot_pointcloud_preprocessing/pointcloud_preprocessor.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import time
import numpy as np
import struct
from scipy.spatial import cKDTree


class PointCloudPreprocessor(Node):
    """
    Point cloud preprocessing node that filters and cleans ZED2i point cloud data.
    Phase 2: Statistical outlier removal with k-nearest neighbor filtering.
    """

    def __init__(self):
        super().__init__('pointcloud_preprocessor')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/zed2i/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('output_topic', '/cleaned_point_cloud')
        self.declare_parameter('stats_interval', 10.0)  # seconds
        self.declare_parameter('enable_timing', True)
        
        # Phase 2: Statistical Outlier Removal parameters
        self.declare_parameter('statistical_outlier.enabled', True)
        self.declare_parameter('statistical_outlier.k_neighbors', 50)
        self.declare_parameter('statistical_outlier.stddev_threshold', 2.0)
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.stats_interval = self.get_parameter('stats_interval').get_parameter_value().double_value
        self.enable_timing = self.get_parameter('enable_timing').get_parameter_value().bool_value
        
        # Phase 2: Statistical outlier removal parameters
        self.statistical_enabled = self.get_parameter('statistical_outlier.enabled').get_parameter_value().bool_value
        self.k_neighbors = self.get_parameter('statistical_outlier.k_neighbors').get_parameter_value().integer_value
        self.stddev_threshold = self.get_parameter('statistical_outlier.stddev_threshold').get_parameter_value().double_value
        
        # Statistics tracking
        self.message_count = 0
        self.total_processing_time = 0.0
        self.max_processing_time = 0.0
        self.min_processing_time = float('inf')
        self.last_stats_time = time.time()
        self.points_removed_count = 0
        self.total_input_points = 0
        
        # QoS Profile for point cloud data - match RViz expectations
        # RViz expects RELIABLE reliability for point clouds
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publisher for cleaned point cloud
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile
        )
        
        # Subscriber to raw point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            qos_profile
        )
        
        # Timer for statistics reporting
        self.stats_timer = self.create_timer(
            self.stats_interval,
            self.report_statistics
        )
        
        self.get_logger().info(f'Point Cloud Preprocessor initialized')
        self.get_logger().info(f'  Input topic: {input_topic}')
        self.get_logger().info(f'  Output topic: {output_topic}')
        self.get_logger().info(f'  Timing enabled: {self.enable_timing}')
        self.get_logger().info(f'  Stats interval: {self.stats_interval}s')
        self.get_logger().info(f'  Statistical outlier removal: {self.statistical_enabled}')
        if self.statistical_enabled:
            self.get_logger().info(f'    K neighbors: {self.k_neighbors}')
            self.get_logger().info(f'    Std dev threshold: {self.stddev_threshold}')

    def pointcloud_callback(self, msg):
        """
        Process incoming point cloud data.
        Phase 2: Statistical outlier removal with k-nearest neighbor filtering.
        """
        start_time = time.time() if self.enable_timing else 0.0
        
        try:
            if self.statistical_enabled:
                # Phase 2: Apply statistical outlier removal
                processed_msg = self.apply_statistical_outlier_removal(msg)
            else:
                # Phase 1: Direct pass-through 
                processed_msg = self.passthrough_processing(msg)
            
            # Publish processed point cloud
            self.publisher.publish(processed_msg)
            
            # Update statistics
            if self.enable_timing:
                processing_time = time.time() - start_time
                self.update_statistics(processing_time)
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def apply_statistical_outlier_removal(self, msg):
        """
        Phase 2: Apply statistical outlier removal to point cloud.
        Uses k-nearest neighbor analysis to identify and remove outlier points.
        """
        # Extract point cloud data
        points = self.extract_xyz_points(msg)
        
        if len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return msg
            
        # Track input statistics
        original_count = len(points)
        self.total_input_points += original_count
        
        # Apply statistical outlier removal
        inlier_mask = self.statistical_outlier_filter(points)
        filtered_points = points[inlier_mask]
        
        # Track removal statistics
        removed_count = original_count - len(filtered_points)
        self.points_removed_count += removed_count
        
        # Create new point cloud message with filtered points
        filtered_msg = self.create_pointcloud_message(filtered_points, msg)
        
        return filtered_msg
    
    def passthrough_processing(self, msg):
        """
        Phase 1: Direct pass-through processing.
        Just update timestamp and republish the same message.
        """
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg
        
    def extract_xyz_points(self, pointcloud_msg):
        """
        Extract XYZ coordinates from PointCloud2 message.
        Assumes standard XYZ format with 32-bit float values.
        """
        # Check if point cloud has XYZ fields
        xyz_offset = None
        point_step = pointcloud_msg.point_step
        
        # Find XYZ field offsets
        x_offset = y_offset = z_offset = None
        for field in pointcloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            self.get_logger().error('Point cloud missing XYZ fields')
            return np.array([])
        
        # Extract binary data
        data = pointcloud_msg.data
        
        # Calculate number of points
        num_points = len(data) // point_step
        
        # Extract XYZ coordinates
        points = []
        for i in range(num_points):
            base_offset = i * point_step
            
            # Extract X, Y, Z as 32-bit floats
            x = struct.unpack('<f', data[base_offset + x_offset:base_offset + x_offset + 4])[0]
            y = struct.unpack('<f', data[base_offset + y_offset:base_offset + y_offset + 4])[0] 
            z = struct.unpack('<f', data[base_offset + z_offset:base_offset + z_offset + 4])[0]
            
            # Filter out invalid points (NaN, inf)
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])
        
        return np.array(points)
    
    def statistical_outlier_filter(self, points):
        """
        Apply statistical outlier removal using k-nearest neighbor analysis.
        Returns boolean mask of inlier points.
        """
        if len(points) < self.k_neighbors + 1:
            # Not enough points for k-neighbor analysis
            return np.ones(len(points), dtype=bool)
        
        # Build KD-tree for fast nearest neighbor search
        tree = cKDTree(points)
        
        # Find k nearest neighbors for each point (excluding self)
        distances, indices = tree.query(points, k=self.k_neighbors + 1)
        
        # Remove self-distance (first column) and compute mean distances
        neighbor_distances = distances[:, 1:]  # Exclude self
        mean_distances = np.mean(neighbor_distances, axis=1)
        
        # Compute statistics of mean distances
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        
        # Define outlier threshold
        threshold = global_mean + self.stddev_threshold * global_std
        
        # Create inlier mask
        inlier_mask = mean_distances <= threshold
        
        return inlier_mask
    
    def create_pointcloud_message(self, points, original_msg):
        """
        Create a new PointCloud2 message from filtered points.
        Maintains original message structure but with filtered data.
        """
        if len(points) == 0:
            # Create empty point cloud
            new_msg = PointCloud2()
            new_msg.header = original_msg.header
            new_msg.header.stamp = self.get_clock().now().to_msg()
            new_msg.height = 1
            new_msg.width = 0
            new_msg.fields = original_msg.fields
            new_msg.is_bigendian = original_msg.is_bigendian
            new_msg.point_step = original_msg.point_step
            new_msg.row_step = 0
            new_msg.data = b''
            new_msg.is_dense = True
            return new_msg
        
        # Create new message with same structure
        new_msg = PointCloud2()
        new_msg.header = original_msg.header
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.height = 1
        new_msg.width = len(points)
        new_msg.fields = original_msg.fields
        new_msg.is_bigendian = original_msg.is_bigendian
        new_msg.point_step = original_msg.point_step
        new_msg.row_step = new_msg.point_step * new_msg.width
        new_msg.is_dense = True
        
        # Build data array
        point_step = original_msg.point_step
        data = bytearray(len(points) * point_step)
        
        # Find field offsets (same as extract function)
        x_offset = y_offset = z_offset = None
        for field in original_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        # Fill in XYZ data for each point
        for i, point in enumerate(points):
            base_offset = i * point_step
            
            # Pack XYZ as 32-bit floats
            data[base_offset + x_offset:base_offset + x_offset + 4] = struct.pack('<f', point[0])
            data[base_offset + y_offset:base_offset + y_offset + 4] = struct.pack('<f', point[1])
            data[base_offset + z_offset:base_offset + z_offset + 4] = struct.pack('<f', point[2])
        
        new_msg.data = bytes(data)
        
        return new_msg
    
    def update_statistics(self, processing_time):
        """Update processing time statistics."""
        self.message_count += 1
        self.total_processing_time += processing_time
        
        if processing_time > self.max_processing_time:
            self.max_processing_time = processing_time
        if processing_time < self.min_processing_time:
            self.min_processing_time = processing_time
    
    def report_statistics(self):
        """Report processing statistics periodically."""
        current_time = time.time()
        time_elapsed = current_time - self.last_stats_time
        
        if self.message_count > 0:
            avg_processing_time = self.total_processing_time / self.message_count
            message_rate = self.message_count / time_elapsed
            
            stats_msg = (
                f'Point Cloud Processing Stats (last {self.stats_interval}s):\n'
                f'  Messages processed: {self.message_count}\n'
                f'  Message rate: {message_rate:.2f} Hz\n'
                f'  Avg processing time: {avg_processing_time*1000:.2f} ms\n'
                f'  Min processing time: {self.min_processing_time*1000:.2f} ms\n'
                f'  Max processing time: {self.max_processing_time*1000:.2f} ms'
            )
            
            # Add filtering statistics if statistical filtering is enabled
            if self.statistical_enabled and self.total_input_points > 0:
                removal_rate = (self.points_removed_count / self.total_input_points) * 100
                avg_points_per_msg = self.total_input_points / self.message_count
                avg_removed_per_msg = self.points_removed_count / self.message_count
                
                stats_msg += (
                    f'\n  Statistical Outlier Removal:\n'
                    f'    Total points processed: {self.total_input_points}\n'
                    f'    Points removed: {self.points_removed_count} ({removal_rate:.2f}%)\n'
                    f'    Avg points per message: {avg_points_per_msg:.0f}\n'
                    f'    Avg removed per message: {avg_removed_per_msg:.1f}'
                )
            
            self.get_logger().info(stats_msg)
        else:
            self.get_logger().warn('No point cloud messages received in the last interval')
        
        # Reset statistics for next interval
        self.message_count = 0
        self.total_processing_time = 0.0
        self.max_processing_time = 0.0
        self.min_processing_time = float('inf')
        self.points_removed_count = 0
        self.total_input_points = 0
        self.last_stats_time = current_time


def main(args=None):
    """Main entry point for the point cloud preprocessor node."""
    rclpy.init(args=args)
    
    try:
        node = PointCloudPreprocessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in point cloud preprocessor: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()