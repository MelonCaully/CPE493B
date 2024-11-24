#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import csv
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
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
        # Extract the current pose of the vehicle
        x_vehicle = pose_msg.pose.position.x
        y_vehicle = pose_msg.pose.position.y
        theta_vehicle = self.euler_from_quaternion(pose_msg.pose.orientation)

        # Find the closest waypoint within the lookahead distance
        goal_point = self.find_closest_waypoint(x_vehicle, y_vehicle)

        # Transform goal point to vehicle frame of reference
        delta_x = goal_point[0] - x_vehicle
        delta_y = goal_point[1] - y_vehicle
        distance = np.sqrt(delta_x**2 + delta_y**2)
        angle_to_goal = np.arctan2(delta_y, delta_x) - theta_vehicle

        # Calculate the steering angle using Pure Pursuit
        curvature = 2 * np.sin(angle_to_goal) / distance
        steering_angle = np.arctan(self.wheelbase * curvature)

        # Publish the drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = np.clip(steering_angle, -0.418, 0.418)  # Steering angle limits
        drive_msg.drive.speed = 1.0  # You can adjust this based on the desired speed
        self.drive_pub.publish(drive_msg)

    def find_closest_waypoint(self, x_vehicle, y_vehicle):
        # Find the closest waypoint within the lookahead distance
        closest_distance = float('inf')
        closest_waypoint = None
        for wp in self.waypoints:
            x_wp, y_wp, _ = wp
            distance = np.sqrt((x_vehicle - x_wp)**2 + (y_vehicle - y_wp)**2)
            if distance < closest_distance and distance <= self.lookahead_distance:
                closest_distance = distance
                closest_waypoint = wp
        return closest_waypoint

    def euler_from_quaternion(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.atan2(t3, t4)

        return yaw_z  # We only need yaw for pure pursuit

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
