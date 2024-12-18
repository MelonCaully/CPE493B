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

        self.braking_speed = 0.0

        self.gap_follow_speed = 0.0
        self.gap_follow_steering_angle = 0.0

    def pure_pursuit_callback(self, msg):
        self.pure_pursuit_msg = msg
        self.publish_drive_command()

    def emergency_braking_callback(self, msg):
        self.braking_msg = msg
        self.braking_speed = msg.drive.speed
        self.publish_drive_command()

    def gap_follow_callback(self, msg):
        self.gap_follow_msg = msg
        self.gap_follow_speed = msg.drive.speed
        self.gap_follow_steering_angle = msg.drive.steering_angle

        self.publish_drive_command()
        
    def publish_drive_command(self):
        # Priority logic: Pure Pursuit > Gap Follow > Emergency Braking
        if self.braking_msg is None and self.gap_follow_msg is None and self.pure_pursuit_msg is None:
            self.get_logger().info('No messages recieved yet')
            return

        if self.braking_speed < 1.0 and self.gap_follow_speed < 1.5:
            self.get_logger().info('Emergency Braking is in control')

            msg = AckermannDriveStamped()
            msg.drive.speed = self.braking_speed
            msg.drive.steering_angle = self.gap_follow_steering_angle

            self.drive_pub.publish(msg)

        elif self.gap_follow_speed < 1.5:            
            self.get_logger().info('Gap Follow is in control')
            self.drive_pub.publish(self.gap_follow_msg)

        else:
            self.get_logger().info('Pure Pursuit is in control')
            self.drive_pub.publish(self.pure_pursuit_msg)


def main(args=None):
    rclpy.init(args=args)
    print('Filter Node Initialized')
    filter_node = FilterNode()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
