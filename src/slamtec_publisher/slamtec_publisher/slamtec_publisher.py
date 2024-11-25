#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
from .slamtec import SlamtecMapper
import time
import numpy as np
import argparse

class SlamtecPublisher(Node):


    def __init__(self):
        super().__init__('slamtec_publisher')

        # Declare parameters
        self.declare_parameter('host', '192.168.11.1')
        self.declare_parameter('port', 1445)
        self.declare_parameter('publish_scan', True)

        # Get parameters
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        publish_scan = self.get_parameter('publish_scan').value

        # Initialize TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Set transform frame names
        self.map_frame = 'map'
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.plate_frame = 'plate'
        self.lidar_frame = 'M2M2_LIDAR'

        # Create publishers
        # Should probably implement this soon (QoS)
        # sensor_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.create_timer(0.1, self.publish_scan)

        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.create_timer(0.5, self.publish_map)

        # self.pose_publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        # self.create_timer(0.1, self.publish_pose)

        # Initialize Slamtec Mapper
        self.slamtec = SlamtecMapper(host=host, port=port)

        # Publish static transforms
        self.publish_static_transforms()


    # Publish all static transforms needed (odom and map)
    def publish_static_transforms(self):
        # Static transform: map -> odom
        static_transform_map_to_odom = TransformStamped()
        static_transform_map_to_odom.header.stamp = self.get_clock().now().to_msg()
        static_transform_map_to_odom.header.frame_id = self.map_frame
        static_transform_map_to_odom.child_frame_id = self.odom_frame
        static_transform_map_to_odom.transform.translation.x = 0.0
        static_transform_map_to_odom.transform.translation.y = 0.0
        static_transform_map_to_odom.transform.translation.z = 0.0
        static_transform_map_to_odom.transform.rotation.x = 0.0
        static_transform_map_to_odom.transform.rotation.y = 0.0
        static_transform_map_to_odom.transform.rotation.z = 0.0
        static_transform_map_to_odom.transform.rotation.w = 1.0

        # Static transform: base_link -> plate
        static_transform_base_link_to_plate = TransformStamped()
        static_transform_base_link_to_plate.header.stamp = self.get_clock().now().to_msg()
        static_transform_base_link_to_plate.header.frame_id = self.base_frame
        static_transform_base_link_to_plate.child_frame_id = self.plate_frame
        static_transform_base_link_to_plate.transform.translation.x = 0.0
        static_transform_base_link_to_plate.transform.translation.y = 0.0
        static_transform_base_link_to_plate.transform.translation.z = 0.0
        static_transform_base_link_to_plate.transform.rotation.x = 0.0
        static_transform_base_link_to_plate.transform.rotation.y = 0.0
        static_transform_base_link_to_plate.transform.rotation.z = 0.0
        static_transform_base_link_to_plate.transform.rotation.w = 1.0

        # Publish both static transforms
        self.static_tf_broadcaster.sendTransform([
            static_transform_map_to_odom,
            static_transform_base_link_to_plate
        ])

        self.get_logger().info('Published static transforms: map -> odom and base_link -> plate')


    def publish_scan(self):
        # Get laser scan data
        scan_data = self.slamtec.get_laser_scan(valid_only=True)

        # Create LaserScan message
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.lidar_frame

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
        # Get map data from M2M2
        map_data = self.slamtec.get_map_data()
        
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        
        # Set map metadata
        msg.info.resolution = 0.05  # Adjust according to your map's resolution
        msg.info.width = map_data['dimension_x']
        msg.info.height = map_data['dimension_y']

        # Center map on robot's initial position
        msg.info.origin.position.x = -msg.info.width * msg.info.resolution / 2.0
        msg.info.origin.position.y = -msg.info.height * msg.info.resolution / 2.0
        msg.info.origin.orientation.w = 1.0
        
        # Convert map data to 1D grid
        grid_data = []
        for x in range(msg.info.height):
            for y in range(msg.info.width):
                # Adjust indices to match the coordinate system
                value = map_data["map_data"][map_data['dimension_y'] - x][y]
                if value < 0:
                    grid_data.append(-1)  # Unknown
                else:
                    # Convert to occupancy value between 0 and 100
                    occupancy_value = max(0, min(100, int(int(value) / 250 * 100)))
                    grid_data.append(occupancy_value)
        
        msg.data = grid_data
        self.get_logger().info(f'Publishing map with {len(grid_data)} cells')
        self.map_publisher_.publish(msg)


    def publish_pose(self):
        pose_data = self.slamtec.get_pose()
        
        # Transform: odom -> base_link (dynamic)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pose_data['x']
        t.transform.translation.y = pose_data['y']
        t.transform.rotation.z = math.sin(pose_data['yaw'] / 2.0)
        t.transform.rotation.w = math.cos(pose_data['yaw'] / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SlamtecPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()