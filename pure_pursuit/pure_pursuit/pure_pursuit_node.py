#!/usr/bin/env python3
import rclpy
import csv
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.pose_subscriber = self.create_subscription(PoseStamped, '/car_pose', self.pose_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.declare_parameter('lookahead_distance', 10)
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.waypoints = self.load_waypoints('/path/to/waypoints.csv')

    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                waypoints.append([float(row[0]), float(row[1])])
        return waypoints

    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture
        x, y = pose_msg.pose.position.x, pose_msg.pose.position.y

        goal = None
        min_distance = float('inf')
        for waypoint in self.waypoints:
            distance = np.sqrt((waypoint[0] - x)**2 + (waypoint[1] - y)**2)
            if distance >= self.lookahead_distance and distance < min_distance:
                min_distance = distance
                goal = waypoint

        if not goal:
            self.get_logger().warn('No valid waypoint found within lookahead distance.')
            return

        # TODO: transform goal point to vehicle frame of reference
        dx = goal[0] - x
        dy = goal[1] - y
        theta = np.arctan2(dy, dx) - pose_msg.pose.orientation.z  # Assuming 2D

        # TODO: calculate curvature/steering angle
        curvature = 2 * abs(dy) / (self.lookahead_distance**2)
        steering_angle = np.clip(np.arctan(curvature), -0.4189, 0.4189)  # ~24 degrees max

        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = 1.0  # Adjust speed as necessary
        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()