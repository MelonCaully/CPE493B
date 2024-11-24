#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import csv

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # ROS Parameters
        self.declare_parameter("lookahead_distance", 1.5)
        self.declare_parameter("wheelbase", 0.325)
        self.declare_parameter("waypoints_file", "data.csv")
        
        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.waypoints_file = self.get_parameter("waypoints_file").value
        
        # Load waypoints from file
        self.waypoints = self.load_waypoints(self.waypoints_file)
        
        # Publishers and Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.waypoints_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Visualization
        self.visualize_waypoints()

    def load_waypoints(self, filename):
        waypoints = []
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                waypoints.append((float(row[0]), float(row[1]), float(row[2])))  # Load x, y, z
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints

    def visualize_waypoints(self):
        marker_array = MarkerArray()
        for i, (x, y, z) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"  # Ensure this matches your setup
            marker.header.stamp = self.get_clock().now().to_msg()
            
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Marker scale 
            marker.scale.x = 0.1  # Adjust marker size as needed
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Marker color 
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            # Set position with 3D coordinates
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.id = i
            marker_array.markers.append(marker)
        
        # Publish markers
        self.waypoints_pub.publish(marker_array)


    def pose_callback(self, pose_msg):
        x = 0

    def find_goal_waypoint(self, current_x, current_y):
        x = 0

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
