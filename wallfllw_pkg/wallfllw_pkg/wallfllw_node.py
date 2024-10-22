#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from math import isfinite
import time
import atexit

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # PID constants (tune these values)
        self.kp = 0.5
        self.ki = 0.01
        self.kd = 0.0

        # Initialize state
        self.prev_error = 0.0
        self.integral = 0.0

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
        # Convert angle to an index in the range_data array
        # Assuming a standard LiDAR, where angle_min = -135 and angle_max = 135 degrees
        angle_min = -135.0
        angle_max = 135.0
        num_ranges = len(range_data)
        
        # Find the index that corresponds to the desired angle
        index = int((angle - angle_min) / (angle_max - angle_min) * (num_ranges - 1))
        
        # Get the range at the index, handle NaN and inf
        if index >= 0 and index < num_ranges:
            range_value = range_data[index]
            if not np.isfinite(range_value):  # Check for NaN or inf
                return 0.0
            return range_value
        return 0.0  # Return 0 if index is out of bounds

    def get_error(self, range_data, desired_dist):
        """
        Calculate the error between the car's current distance to the wall and the desired distance.

        Args:
            range_data: array of ranges from the LiDAR scan
            desired_dist: the desired distance to the wall (in meters)

        Returns:
            error: the error term to be used for PID control
        """
        # Get the ranges 'a' and 'b' from the laser scan
        a = self.get_range(range_data, 45)
        b = self.get_range(range_data, 90)

        if not (np.isfinite(a) and np.isfinite(b)) or a == 0:
            return 0.0
        
        # Set the angle θ between the two beams (in radians)
        theta = np.radians(45)  # Example value for θ (can be adjusted)
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))

        # Calculate the current distance to the wall (Dt)
        Dt = b * np.cos(alpha)
        Dt_plus_1 = Dt + 1.0 * np.sin(alpha)

        # Calculate the error as the difference between the desired and actual distance
        return desired_dist - Dt_plus_1 

    def pid_control(self, error, velocity):
        # PID formula
        derivative = (error - self.prev_error) / 0.1
        self.integral += error * 0.1

        angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error

        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle

        # Safety adjustment for speed based on proximity to the wall
        if abs(angle) <= np.radians(10):
            drive_msg.drive.speed = 1.5
        elif abs(angle) <= np.radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5  # Slow down if steering sharply

        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        desired_dist = 1.0  # Desired distance to the wall (e.g., 1 meter)
        error = self.get_error(msg.ranges, desired_dist)
        self.pid_control(error, msg.ranges)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()