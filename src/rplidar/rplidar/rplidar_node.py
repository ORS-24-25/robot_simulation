import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from adafruit_rplidar import RPLidar
from math import floor, pi

class AdafruitRPLidarNode(Node):
    def __init__(self):
        super().__init__('adafruit_rplidar_node')

        # Create QoS profile
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher_ = self.create_publisher(LaserScan, 'scan', qos_profile)

        # Initialize RPLidar (adjust port as needed)
        self.lidar = RPLidar(motor_pin=None, port='/dev/ttyUSB0', timeout=3)

        # Prepare a LaserScan message template
        self.scan_data = [0.0]*360
        self.scan_msg = LaserScan()
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_msg.header.frame_id = 'laser_frame'
        self.scan_msg.angle_min = -pi
        self.scan_msg.angle_max = pi
        self.scan_msg.angle_increment = (2*pi)/360
        self.scan_msg.range_min = 0.12
        self.scan_msg.range_max = 8.0

        # Create a timer that executes the callback at 10 Hz
        self.scan_timer = self.create_timer(0.2, self.scan_callback)

    def scan_callback(self):
        # while (self.lidar.motor == False):
        #     self.lidar.connect()
        #if self.lidar.health[1] != 0 :
        #self.get_logger().debug("motor is moving")
        # try:
        #     if not self.scan_msg.ranges:
        #         raise ValueError("empty scan msg")
        # except Exception as e:
        #     self.get_logger().info("RPLIDAR error")
            
            

        # Read one scan set
        for scan in self.lidar.iter_scans():
            for (_, angle, dist_mm) in scan:
                idx = min(359, floor(angle))
                self.scan_data[idx] = dist_mm / 1000.0  # Convert mm to meters

            self.scan_msg.header.stamp = self.get_clock().now().to_msg()
            self.scan_msg.ranges = self.scan_data
            self.publisher_.publish(self.scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdafruitRPLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
