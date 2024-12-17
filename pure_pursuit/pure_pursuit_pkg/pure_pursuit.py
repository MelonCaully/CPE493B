#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        # Create a publisher for driving commands
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            'pure_pursuit/drive',
            10
        )

        # Create a subscriber to Pose data
        self.subscription_odom = self.create_subscription(
            Odometry, 
            '/ego_racecar/odom', 
            self.odom_callback, 
            10
        )
        
        # Create a publisher to Marker data
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )

        self.min_lookahead = 1.25
        self.max_lookahead = 8
        self.current_waypoint_idx = 0
        self.markers = self.load_markers(csv_path=os.getcwd()+'/src/CPE493B/pure_pursuit/data.csv')
        self.markers = list(np.float_(self.markers))
        self.publish_markers()        

    def load_markers(self, csv_path):
        markers = [] 
        with open(csv_path, 'r') as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                markers.append([row[0], row[1], row[2]])

        return markers
    
    def publish_markers(self):
        marker_array = MarkerArray()
        for idx, (x, y, z) in enumerate(self.markers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'markers'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
        self.get_logger().info('Array Published')

    
    def find_goal_waypoint(self, curr_x, curr_y):
        # Given a list of waypoints
        for idx, i in enumerate(self.markers[self.current_waypoint_idx:], start=self.current_waypoint_idx + 1):
            # if at the end of the markers, go to the beginning and keep looking
            if idx >= len(self.markers) - 1: self.current_waypoint_idx = idx = 0 
            else: 
                # Get first distance of the markers from current position
                distance = np.sqrt((self.markers[idx][0] - curr_x)**2 + (self.markers[idx][1] - curr_y)**2)
                
                # Go up the waypoint until you get the one that is one Looahead Dist away from car
                if distance > self.min_lookahead and distance < self.max_lookahead:
                    return idx
                
        return 0
    
    def find_closest_waypoint(self, curr_x, curr_y):
        min_distance = float('inf')  # Start with an infinitely large distance
        closest_idx = -1  # Initialize with an invalid index

        for idx, (wp_x, wp_y, _) in enumerate(self.markers):
            # Calculate the distance from current pos to next closest waypoint
            distance = np.sqrt((wp_x - curr_x) ** 2 + (wp_y - curr_y) ** 2)
            
            # Update if this waypoint is closer than the previously found one
            if distance < min_distance:
                min_distance = distance
                closest_idx = idx

        # return once found closest index
        return closest_idx


    def odom_callback(self, odom_msg):
        # Current position and Orientation of the car
        curr_x, curr_y, orientation = float(odom_msg.pose.pose.position.x), float(odom_msg.pose.pose.position.y), odom_msg.pose.pose.orientation

        # Calculating yaw of theta, where the car is going
        theta = np.arctan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        # TODO: find the current waypoint to track using methods mentioned in lecture
        self.current_waypoint_idx = self.find_closest_waypoint(curr_x, curr_y)
        goal_idx = self.find_goal_waypoint(curr_x, curr_y)
        goal_x, goal_y = self.markers[goal_idx][0], self.markers[goal_idx][1]

        # TODO: transform goal point to vehicle frame of reference
        dx = goal_x - curr_x  
        dy = goal_y - curr_y

        local_x = dx * np.cos(-theta) - dy * np.sin(-theta)
        local_y = dx * np.sin(-theta) + dy * np.cos(-theta)
        goal_dist = (local_x**2 + local_y**2)
        
        # TODO: calculate curvature/steering angle
        curvature = (2 * local_y) / goal_dist
        steering_angle = np.arctan(curvature)

        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = np.clip(steering_angle, -.1, .1)
        drive_msg.drive.speed = 1.5  # Adjust speed as needed
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