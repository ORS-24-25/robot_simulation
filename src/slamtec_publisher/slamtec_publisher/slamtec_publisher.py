#!/usr/bin/env python3
"""slamtec_publisher.py
This module contains the SlamtecPublisher class, which is a ROS2 node that publishes
data from a Slamtec M2M2 Mapper.

Classes:
    SlamtecPublisher: A ROS2 node that publishes data from a Slamtec M2M2 Mapper.
Functions:
    main(args=None): Entry point for the SlamtecPublisher node.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
from .slamtec import SlamtecMapper

class SlamtecPublisher(Node):
    """SlamtecPublisher class for publishing Slamtec Mapper data.
    This node publishes laser scan data, occupancy grid maps, and the robot's pose.
    It also handles the broadcasting of static and dynamic transforms.
    Attributes:
        tf_broadcaster (TransformBroadcaster): Broadcasts dynamic transforms.
        static_tf_broadcaster (StaticTransformBroadcaster): Broadcasts static transforms.
        map_frame (str): Name of the map frame.
        odom_frame (str): Name of the odom frame.
        base_frame (str): Name of the base frame.
        plate_frame (str): Name of the plate frame.
        lidar_frame (str): Name of the lidar frame.
        scan_publisher_ (Publisher): Publishes laser scan data.
        map_publisher_ (Publisher): Publishes occupancy grid maps.
        pose_publisher_ (Publisher): Publishes the robot's pose.
        slamtec (SlamtecMapper): Interface to the Slamtec Mapper.

    Methods:
        __init__(self): Initialize the SlamtecPublisher node.
        publish_static_transforms(self): Publish static transforms between frames.
        publish_scan(self): Publish laser scan data to /scan.
        publish_map(self): Publish occupancy grid map to /map.
        publish_pose(self): Publish the robot's pose.
    """

    def __init__(self):
        """
        Initialize the SlamtecPublisher node.

        This method declares and retrieves parameters, initializes publishers and timers,
        and sets up the Slamtec Mapper.
        """
        super().__init__('slamtec_publisher')

        # Declare parameters
        self.declare_parameter('host', '192.168.11.1')
        self.declare_parameter('port', 1445)

        # Get parameters
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

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
        # scan_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.scan_publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.create_timer(0.1, self.publish_scan)

        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.create_timer(0.5, self.publish_map)

        self.pose_publisher_ = self.create_publisher(PoseStamped, 'pose', pose_qos)
        self.create_timer(0.1, self.publish_pose)

        # Initialize Slamtec Mapper
        self.slamtec = SlamtecMapper(host=host, port=port)
        self.slamtec.clear_map()

        # Publish static transforms
        self.publish_static_transforms()

        # Check connection. \\ How often to do this?
        self.create_timer(1, self.slamtec.check_connection)

        self.get_logger().info("slamtec_publisher node started succesfully")

    def publish_static_transforms(self):
        """
        Publish static transforms between frames.

        This method publishes the static transform between the base_link and plate frames.
        """
        now = self.get_clock().now().to_msg()

        # Static transform: base_link -> plate
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = self.base_frame
        t1.child_frame_id = self.plate_frame
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        # Static transform: odom -> base_link
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = self.odom_frame
        t2.child_frame_id = self.base_frame
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # Publish static transforms
        self.static_tf_broadcaster.sendTransform(
            [t1, t2]
        )

        # self.get_logger().info('Published static transforms: base_link -> plate and odom -> base_link')

    def publish_scan(self):
        """
        Publish laser scan data to /scan.

        This method retrieves laser scan data from the Slamtec Mapper and publishes it
        as a LaserScan message.
        """
        # Get laser scan data
        try:
            scan_data = self.slamtec.get_laser_scan(valid_only=True)
        except:
            self.slamtec.check_connection()
            return

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
        """
        Publish occupancy grid map to /map.

        This method retrieves map data from the Slamtec Mapper and publishes it
        as an OccupancyGrid message.
        """
        # Get map data from M2M2
        try:
            map_data = self.slamtec.get_map_data()
        except BaseException as e:
            self.get_logger().error(f"Failed to get map data: {e}")
            self.slamtec.check_connection()
            return
        
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        
        # Set map metadata
        msg.info.resolution = 0.05  # Adjust according to your map's resolution
        msg.info.width = map_data['dimension_x']
        msg.info.height = map_data['dimension_y']

        # Center map on robot's initial position
        pose = self.slamtec.get_pose()
        # Transform from odom to map
        # msg.info.origin.position.x = pose['x'] - (msg.info.width * msg.info.resolution / 2.0)
        msg.info.origin.position.x = 0.0
        # msg.info.origin.position.y = pose['y'] - (msg.info.height * msg.info.resolution / 2.0)
        msg.info.origin.position.y = 0.0
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
        # self.get_logger().info(f'Publishing map with {len(grid_data)} cells')
        self.map_publisher_.publish(msg)

    def publish_pose(self):
        """
        Publish the robot's pose as both a TransformStamped and a PoseStamped message.
        It also broadcasts the dynamic transform between the map and odom frames.
        """
        try:
            pose_data = self.slamtec.get_pose()
        except BaseException as e:
            self.get_logger().error(f"Failed to get pose data: {e}")
            self.slamtec.check_connection()
            return
        
        if not pose_data:
            self.get_logger().warn("Pose data is empty.")
            return

        # Extract pose data
        try:
            x = float(pose_data['x'])
            y = float(pose_data['y'])
            yaw = float(pose_data['yaw'])
        except (KeyError, ValueError) as e:
            self.get_logger().error(f"Invalid pose data format: {e}")
            return
        
        # --- Publish TransformStamped: (dynamic map -> odom) ---
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        # Calculate the rotation quaternion from yaw
        siny_cosp = math.sin(yaw / 2.0)
        cosy_cosp = math.cos(yaw / 2.0)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = siny_cosp
        transform.transform.rotation.w = cosy_cosp

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)

        # --- Publish PoseStamped ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame  # Frame relative to which pose is defined
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = siny_cosp
        pose_msg.pose.orientation.w = cosy_cosp

        # Publish PoseStamped
        self.pose_publisher_.publish(pose_msg)
        # self.get_logger().debug("Published PoseStamped message")

def main(args=None):
    """
    Entry point for the SlamtecPublisher node.

    This function initializes the ROS2 node and starts spinning.
    """
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