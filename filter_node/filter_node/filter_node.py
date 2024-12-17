#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')

        # Subscribers to input nodes
        self.pure_pursuit_sub = self.create_subscription(
            AckermannDriveStamped, 
            'pure_pursuit/drive', 
            self.pure_pursuit_callback, 
            10
        )

        self.emergency_braking_sub = self.create_subscription(
            AckermannDriveStamped, 
            'safety_node/drive', 
            self.emergency_braking_callback, 
            10
        )
        self.gap_follow_sub = self.create_subscription(
            AckermannDriveStamped, 
            'gap_follow/drive', 
            self.gap_follow_callback, 
            10
        )

        # Publisher for the final drive command
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            '/drive', 
            10
        )

        # Variables to store the latest messages from each input node
        self.pure_pursuit_msg = None
        self.braking_msg = None
        self.gap_follow_msg = None

    def pure_pursuit_callback(self, msg):
        self.pure_pursuit_msg = msg
        self.publish_drive_command()

    def emergency_braking_callback(self, msg):
        self.braking_msg = msg
        self.publish_drive_command()

    def gap_follow_callback(self, msg):
        self.gap_follow_msg = msg
        self.publish_drive_command()
    
    def get_speed(self, msg):
        if msg is None:
            return 6.0  # Default speed when no message is received
        if msg.drive.speed is not None:
            return float(msg.drive.speed)
        else:
            return 6.0

    def publish_drive_command(self):
        # Priority logic: Pure Pursuit > Gap Follow > Emergency Braking
        # Publish the first available message based on priority

        # If Pure Pursuit message is available, use it
        if self.pure_pursuit_msg is not None:
            self.get_logger().info('Pure Pursuit is in control')
            self.drive_pub.publish(self.pure_pursuit_msg)

        # If Gap Follow message is available and Pure Pursuit message is not
        elif self.gap_follow_msg is not None:
            self.get_logger().info('Gap Follow is in control')
            self.drive_pub.publish(self.gap_follow_msg)

        # If Emergency Braking message is available and others are not
        elif self.braking_msg is not None:
            self.get_logger().info('Emergency Braking is in control')
            self.drive_pub.publish(self.braking_msg)
        else:
            # If no messages are available, don't publish anything
            self.get_logger().info('No messages received yet')


def main(args=None):
    rclpy.init(args=args)
    print('Filter Node Initialized')
    filter_node = FilterNode()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
