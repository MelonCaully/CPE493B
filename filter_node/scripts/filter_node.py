#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

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
        self.braking = self.create_subscription(
            AckermannDriveStamped, 
            'safety_node/drive', 
            self.obstacle_avoidance_callback, 
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
        self.obstacle_avoidance_msg = None
        self.gap_follow_msg = None

    def pure_pursuit_callback(self, msg):
        self.pure_pursuit_msg = msg
        self.publish_drive_command()

    def obstacle_avoidance_callback(self, msg):
        self.obstacle_avoidance_msg = msg
        self.publish_drive_command()

    def gap_follow_callback(self, msg):
        self.gap_follow_msg = msg
        self.publish_drive_command()

    def publish_drive_command(self):
        # Priority logic: Pure Pursuit > Obstacle Avoidance > Gap Follow
        if self.obstacle_avoidance_msg and should_take_control(self.obstacle_avoidance_msg):
            self.get_logger().info('Emergency Braking is in control')
            self.drive_pub.publish(self.obstacle_avoidance_msg)
        elif self.gap_follow_msg and should_take_control(self.gap_follow_msg):
            self.get_logger().info('Gap Follow is in control')
            self.drive_pub.publish(self.gap_follow_msg)
        elif self.pure_pursuit_msg:
            self.get_logger().info('Pure Pursuit is in control')
            self.drive_pub.publish(self.pure_pursuit_msg)

def should_take_control(msg):
    # Example logic: Use obstacle avoidance or gap follow only if critical
    # Replace with actual conditions relevant to your system
    return msg.speed < 0.5  # Example: Low speed implies taking control is critical

def main(args=None):
    rclpy.init(args=args)
    filter_node = FilterNode()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
