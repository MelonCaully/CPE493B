import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscriber for Lidar scan data
        self.scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)

        # Create publisher for drive commands
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # PID gains
        self.kp = 1.0 # Proportional gain
        self.kd = 0.0 # Integral gain
        self.ki = 0.1 # Derivative gain

        # Store history
        self.integral = 0.0
        self.prev_error = 0.0
        # self.error = 

        # Desired distance from the wall (meters)
        self.desired_distance = 1.0
        self.velocity = 1.0 # Desired velocity in m/s

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        angle_index = int((angle - range_data.angle_min) / range_data.angle_increment)

        if angle_index < len(range_data.ranges):
            distance = range_data.ranges[angle_index]
            if np.isfinite(distance):  # Handle NaN and infinity values
                return distance
        return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        a = self.get_range(range_data, 60)  # Beam a at 60 degrees
        b = self.get_range(range_data, 90)  # Beam b at 90 degrees (right side)

        # Calculate alpha (angle between car's x-axis and the wall)
        theta = np.radians(60)
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))

        # Current distance to the wall (Dt)
        dt = b * np.cos(alpha)

        # Project future distance to the wall (Dt+1)
        L = 1.0  # Lookahead distance (you may tune this)
        dt_plus_1 = dt + L * np.sin(alpha)

        # Calculate the error as the difference between desired and actual distance
        error = dist - dt_plus_1
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # PID calculations
        proportional = error
        self.integral += error
        derivative = error - self.prev_error

        # Compute the steering angle
        steering_angle = (self.kp * proportional +
                          self.ki * self.integral +
                          self.kd * derivative)

        self.prev_error = error

        # Create and publish the drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance)
        velocity = self.velocity
        self.pid_control(error, velocity) 


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()