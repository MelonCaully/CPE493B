#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')

        # Publisher to /drive topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'safety_node/drive', 10)

        # Subscribers to /scan and /ego_racecar/odom topics
        self.scan_subscription = self.create_subscription(LaserScan,'scan',self.scan_callback,10)
        self.odom_subscription = self.create_subscription(Odometry,'ego_racecar/odom',self.odom_callback,10)
        
        self.speed = 0. # Vehicle speed

    def odom_callback(self, odom_msg):
        # Updates current speed from odometry message
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Analyzes the LaserScan data to determine if emergency braking is needed.
        ranges = scan_msg.ranges
        num = len(ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        speed = self.speed

        brake_threshold = 1.0 # Tune this to best value through trial and error 

        # Loop through each range reading to analyze potential collisions
        for i in range(num):
            angle = angle_min + i*angle_increment 
            x = speed * np.cos(angle) 

            if x <= 0:
                x = 0 
            else:
                iTTC = ranges[i] / x

            # Check if emergency braking is required
            if x > 0 and iTTC < brake_threshold: 
                self.publish_brake()
    
    def publish_brake(self):
        # Publishes brake command to stop vehicle
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = 0.0
        self.publisher_.publish(new_msg) 

        # Used to show the vehicle is braking
        self.get_logger().info("EMERGENCY BRAKING!")

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()