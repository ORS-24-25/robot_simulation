#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math
from .slamtec import SlamtecMapper
import time
import numpy as np
import argparse

class SlamtecPublisher(Node):
    def __init__(self, host='192.168.11.1', port=1445, debug=False, publish_scan=False):
        super().__init__('slamtec_publisher')

        # Initialize TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set transform frame names
        self.laser_frame = 'M2M2_LIDAR'
        self.base_frame = 'M2M2_LIDAR'
        self.odom_frame = 'M2M2_LIDAR'

        # Create publishers
        if publish_scan:
            print("Publishing only scan data")
        # sensor_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        self.scan_publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.create_timer(0.1, self.publish_scan)

        if not publish_scan:
            print("Publishing scan, map, and pose data")
            self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
            self.pose_publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
            self.create_timer(1.0, self.publish_map)
            self.create_timer(0.1, self.publish_pose)

        # Initialize Slamtec Mapper
        self.slamtec = SlamtecMapper(host=host, port=port)

    def publish_scan(self):
        # Get laser scan data
        scan_data = self.slamtec.get_laser_scan(valid_only=True)

        # Create LaserScan message
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.laser_frame

        # print(scan_data)

        # Convert data
        angles = []
        ranges = []
        intensities = []
        for angle, distance, valid in scan_data:
            if distance < 0.15 or distance > 8.0:
                valid = False
            angles.append(angle)
            ranges.append(distance)
            intensities.append(100.0 if valid else 0.0)

        msg.angle_min = min(angles)
        msg.angle_max = max(angles)
        msg.angle_increment = (msg.angle_max - msg.angle_min) / len(angles)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 8.0
        msg.ranges = ranges
        msg.intensities = intensities

        self.scan_publisher_.publish(msg)

    def publish_map(self):
        map_data = self.slamtec.get_map_data()
        known_data = self.slamtec.get_known_area()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        
        # Convert map data to occupancy grid
        msg.info.resolution = 0.05  # 5cm per pixel
        msg.info.width = known_data['max_x'] - known_data['min_x']
        msg.info.height = known_data['max_y'] - known_data['min_y']
        msg.info.origin.position.x = map_data.get('min_x', 0.0)
        msg.info.origin.position.y = map_data.get('min_y', 0.0)
        
        # Convert 2D map data to 1D array
        grid_data = []
        for y in range(msg.info.height):
            # Convert y index from 0-based to 1-based for map data access
            row = map_data['map_data'].get(y+1, [])  # Use get() with default empty list
            if not row:
                self.get_logger().warn(f"Missing map data for row {y+1}")
                grid_data.extend([-1] * msg.info.width)  # Fill row with unknown
                continue
            for x in range(msg.info.width):
                if x >= len(row):
                    grid_data.append(-1)  # Unknown space
                    continue
                # Clamp values between 0-100
                value = max(0, min(100, int(row[x] * 100)))
                grid_data.append(value)
        
        msg.data = grid_data
        self.map_publisher_.publish(msg)

    def publish_pose(self):
        pose_data = self.slamtec.get_pose()
        
        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = pose_data['x']
        pose_msg.pose.position.y = pose_data['y']
        pose_msg.pose.orientation.z = math.sin(pose_data['yaw'] / 2.0)
        pose_msg.pose.orientation.w = math.cos(pose_data['yaw'] / 2.0)
        self.pose_publisher_.publish(pose_msg)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pose_data['x']
        t.transform.translation.y = pose_data['y']
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    # Start parsing command line arguments
    parser = argparse.ArgumentParser(description='Slamtec Mapper Publisher')
    parser.add_argument('--host', type=str, default='192.168.11.1', help='Slamtec Mapper IP address')
    parser.add_argument('--port', type=int, default=1445, help='Slamtec Mapper port number')
    parser.add_argument('--scan', action='store_true', help='Publish scan data')

    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    publisher = SlamtecPublisher(
        host=parsed_args.host, 
        port=parsed_args.port,
        publish_scan=(True if parsed_args.scan else False),
    )
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()