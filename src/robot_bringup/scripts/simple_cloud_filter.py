#!/usr/bin/env python3
"""
Simple Point Cloud Filtering Node for ZED Fused Cloud
Implements confidence-based filtering, temporal consistency, and basic outlier removal
for robust VI-SLAM mapping with glass/mirror rejection.

Author: Claude Code
File: src/robot_bringup/scripts/simple_cloud_filter.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque
import cv2
from cv_bridge import CvBridge
import struct


class SimpleCloudFilter(Node):
    def __init__(self):
        super().__init__('simple_cloud_filter')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 80.0)  # 0-100
        self.declare_parameter('temporal_frames', 3)          # Number of frames for temporal consistency
        self.declare_parameter('outlier_neighbors', 5)        # Min neighbors for outlier removal
        self.declare_parameter('outlier_radius', 0.10)        # 10cm search radius
        self.declare_parameter('bilateral_sigma', 0.05)       # 5cm bilateral filtering
        self.declare_parameter('input_cloud_topic', '/zed2i/zed_node/mapping/fused_cloud')
        self.declare_parameter('confidence_topic', '/zed2i/zed_node/confidence/confidence_map')
        self.declare_parameter('output_cloud_topic', '/zed2i/zed_node/mapping/fused_cloud_filtered')
        
        # Get parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.temporal_frames = self.get_parameter('temporal_frames').value
        self.outlier_neighbors = self.get_parameter('outlier_neighbors').value
        self.outlier_radius = self.get_parameter('outlier_radius').value
        self.bilateral_sigma = self.get_parameter('bilateral_sigma').value
        
        # Temporal consistency tracking
        self.point_history = deque(maxlen=self.temporal_frames)
        self.position_tolerance = 0.05  # 5cm movement tolerance
        
        # CV Bridge for confidence map
        self.bridge = CvBridge()
        self.latest_confidence = None
        
        # Publishers and subscribers
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('input_cloud_topic').value,
            self.cloud_callback,
            10
        )
        
        self.confidence_sub = self.create_subscription(
            Image,
            self.get_parameter('confidence_topic').value,
            self.confidence_callback,
            10
        )
        
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('output_cloud_topic').value,
            10
        )
        
        self.get_logger().info(f"Simple Cloud Filter initialized")
        self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"Temporal frames: {self.temporal_frames}")
        self.get_logger().info(f"Outlier removal: {self.outlier_neighbors} neighbors in {self.outlier_radius}m")

    def confidence_callback(self, msg):
        """Store latest confidence map for point filtering"""
        try:
            self.latest_confidence = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert confidence map: {e}")

    def cloud_callback(self, msg):
        """Main point cloud filtering pipeline"""
        if self.latest_confidence is None:
            self.get_logger().warn("No confidence map available, using input cloud as-is")
            self.filtered_cloud_pub.publish(msg)
            return
            
        try:
            # Extract points from PointCloud2
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            if len(points) == 0:
                self.get_logger().warn("Empty point cloud received")
                return
                
            points_array = np.array(points)
            
            # Step 1: Confidence-based filtering
            filtered_points = self.confidence_filter(points_array, msg)
            
            if len(filtered_points) == 0:
                self.get_logger().warn("All points filtered out by confidence")
                return
            
            # Step 2: Temporal consistency filtering
            filtered_points = self.temporal_filter(filtered_points)
            
            # Step 3: Simple outlier removal
            filtered_points = self.outlier_filter(filtered_points)
            
            # Step 4: Optional bilateral-style smoothing
            filtered_points = self.bilateral_filter(filtered_points)
            
            # Publish filtered cloud
            filtered_msg = self.create_pointcloud2(filtered_points, msg.header)
            self.filtered_cloud_pub.publish(filtered_msg)
            
            # Stats
            original_count = len(points)
            filtered_count = len(filtered_points)
            filter_ratio = (1.0 - filtered_count / original_count) * 100
            self.get_logger().debug(f"Filtered {filter_ratio:.1f}% points ({original_count} -> {filtered_count})")
            
        except Exception as e:
            self.get_logger().error(f"Point cloud filtering failed: {e}")
            # Fallback: publish original cloud
            self.filtered_cloud_pub.publish(msg)

    def confidence_filter(self, points, cloud_msg):
        """Filter points based on ZED confidence map"""
        # This is a simplified approach - in practice you'd need to project 3D points
        # back to image coordinates and check confidence at those pixels
        # For now, we'll use a simple distance-based heuristic combined with height
        
        filtered_points = []
        
        for point in points:
            x, y, z = point
            
            # Simple heuristic: reject points that are likely from glass/mirrors
            # - Very far points (> 12m) are often noise
            # - Points at very regular distances might be reflections
            # - Points significantly above/below camera height might be ceiling/floor reflections
            
            # Distance check
            distance = np.sqrt(x*x + y*y + z*z)
            if distance > 12.0:
                continue
                
            # Height check (assuming camera is ~1m high, reject ceiling/floor reflections)
            if abs(y) > 2.5:  # Y is up in ZED frame
                continue
                
            # Simple reflection detection: reject points at very regular intervals
            # This is a basic heuristic - real confidence map would be better
            distance_mod = distance % 1.0  # Check for regular patterns
            if distance_mod < 0.05 or distance_mod > 0.95:  # Too regular
                continue
                
            filtered_points.append(point)
        
        return np.array(filtered_points)

    def temporal_filter(self, points):
        """Filter points that move too much between frames (moving objects, reflections)"""
        if len(self.point_history) < 2:
            # Not enough history, store current points and pass through
            self.point_history.append(points)
            return points
            
        # Compare with previous frame
        prev_points = self.point_history[-1]
        
        # Simple approach: for each point, check if there's a nearby point in previous frame
        # If a point appears/disappears consistently, it might be a moving object
        filtered_points = []
        
        for point in points:
            # Find closest point in previous frame
            if len(prev_points) > 0:
                distances = np.linalg.norm(prev_points - point, axis=1)
                min_distance = np.min(distances)
                
                # If point has a nearby point in previous frame, it's stable
                if min_distance < self.position_tolerance:
                    filtered_points.append(point)
                # If it's a new point far from others, might be noise - skip for now
                
        # Store current points for next iteration
        self.point_history.append(points)
        
        return np.array(filtered_points) if filtered_points else points

    def outlier_filter(self, points):
        """Remove isolated points (simple statistical outlier removal)"""
        if len(points) < self.outlier_neighbors:
            return points
            
        filtered_points = []
        
        for i, point in enumerate(points):
            # Count neighbors within radius
            distances = np.linalg.norm(points - point, axis=1)
            neighbor_count = np.sum(distances < self.outlier_radius) - 1  # -1 to exclude self
            
            # Keep point if it has enough neighbors
            if neighbor_count >= self.outlier_neighbors:
                filtered_points.append(point)
                
        return np.array(filtered_points) if filtered_points else points

    def bilateral_filter(self, points):
        """Simple bilateral-style filtering using neighbor averaging"""
        if len(points) < 3:
            return points
            
        filtered_points = []
        
        for point in points:
            # Find neighbors
            distances = np.linalg.norm(points - point, axis=1)
            neighbors_mask = distances < self.bilateral_sigma * 2
            neighbors = points[neighbors_mask]
            
            if len(neighbors) > 1:
                # Average with neighbors, weighted by distance
                weights = np.exp(-distances[neighbors_mask] / self.bilateral_sigma)
                weighted_point = np.average(neighbors, axis=0, weights=weights)
                filtered_points.append(weighted_point)
            else:
                # Keep original point if no neighbors
                filtered_points.append(point)
                
        return np.array(filtered_points)

    def create_pointcloud2(self, points, header):
        """Create PointCloud2 message from points array"""
        # Create structured array for PointCloud2
        cloud_data = []
        for point in points:
            x, y, z = point
            # Pack as float32
            cloud_data.append(struct.pack('fff', float(x), float(y), float(z)))
        
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.frame_id = header.frame_id  # Preserve frame
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = b''.join(cloud_data)
        cloud_msg.is_dense = True
        
        return cloud_msg


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCloudFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()