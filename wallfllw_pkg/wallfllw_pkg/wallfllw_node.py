#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # PID constants (tune these values)
        self.kp = 1.0
        self.ki = 0.001
        self.kd = 0.005

        # Initialize state
        self.prev_error = 0.0
        self.integral = 0.0
        self.error = 0.0
        self.velocity = 0.0
        self.del_time = 0.0
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        self.prev_time = seconds + nanoseconds / 1e9

        self.desired_distance = 1.2
        self.velocity = 1.5

        self.angle_a = np.radians(45)
        self.angle_b = np.radians(90)

    def get_range(self, range_data, angle):
        """
        Get the range (distance) at a specific angle from the LiDAR scan.
        Handles NaN and infinity values by returning 0.0 for them.

        Args:
            range_data: array of LiDAR ranges
            angle: angle in degrees, relative to the car's forward direction

        Returns:
            range: distance at the given angle
        """
        b_index = int((np.radians(90) - range_data.angle_min) / range_data.angle_increment)

        if range_data.angle_min > np.radians(45):
            self.angle_a = range_data.angle_min
            a_index = 0
        else:
            a_index = int((np.radians(45.0) - range_data.angle_min)) / range_data.angle_increment

        a_range = range_data.ranges[a_index] if not np.isinf(range_data.ranges[a_index]) and not np.isnan(range_data.ranges[a_index])
        b_range = range_data.ranges[b_index] if not np.isinf(range_data.ranges[b_index]) and not np.isnan(range_data.ranges[b_index])
        return a_range, b_range

    def get_error(self, range_data, dist):
        """
        Calculate the error between the car's current distance to the wall and the desired distance.

        Args:
            range_data: array of ranges from the LiDAR scan
            desired_dist: the desired distance to the wall (in meters)

        Returns:
            error: the error term to be used for PID control
        """
        a_angle = self.angle_a
        b_angle = self.angle_b

        a_range, _ = self.get_range(range_data, a_angle)
        _, b_range = self.get_range(range_data, b_angle)
        
        theta = b_angle - a_angle
        alpha = np.arctan((a_range * np.cos(theta) - b_range)) / (a_range * np.sin(theta))

        # Calculate the current distance to the wall (Dt)
        Dt = b_range * np.cos(alpha)
        Dt_plus_1 = Dt + 1.0 * np.sin(alpha)

        # Calculate the error as the difference between the desired and actual distance
        return dist - Dt_plus_1 

    def pid_control(self, error, velocity):
        angle = 0.0
        # PID formula
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        current_time = seconds + nanoseconds / 1e9
        del_time = current_time - self.prev_time
        self.integral += self.prev_error + del_time
        derivative = (error - self.prev_error) / del_time
        angle = -(self.kp * error + self.ki * self.integral + self.kd * derivative)
        self.prev_time = current_time

        # Safety adjustment for speed based on proximity to the wall
        if abs(angle) <= np.radians(10):
            velocity = 1.5
        elif abs(angle) <= np.radians(20):
            velocity = 1.0
        else:
            velocity = 0.5

        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        error = self.get_error(msg, self.desired_distance)
        self.pid_control(error, self.velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()