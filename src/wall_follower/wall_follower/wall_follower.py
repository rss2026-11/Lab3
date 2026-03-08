#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower_sim")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS!
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS!
        self.add_on_set_parameters_callback(self.parameters_callback)

        # TODO: Initialize your publishers and subscribers here
        #Publisher for the drive
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        #Subscriber to the LIDAR Scan
        self.laser_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        #Create a line in Rviz
        self.wall_pub = self.create_publisher(Marker, "/wall", 1)
        # TODO: Write your callback functions here
        self.error_sum = 0
        self.prev_error = 0


    def convert_lidar_cartesian(self, msg):
        """
        Convert LaserScan msg to cartesian coordinates
        """
        #Gets the ranges and angles from the LIDAR scan
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # Ensure ranges and angles are the same length
        min_len = min(len(ranges), len(angles))
        ranges = ranges[:min_len]
        angles = angles[:min_len]

        # Convert to Cartesian logic
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        return xs, ys


    def filter_points(self, xs, ys):
        """
        Filter points to keep only the relevant wall points.
        """
        # 1. On the correct side
        # 2. In front of the car, but not too far
        # 3. Not too far to the side to avoid distractions
        MAX_FORWARD_DIST = 15.0
        MIN_FORWARD_DIST = 0.3
        MAX_SIDE_DIST = 3.0
        FRONT_CONE_DIST = 4.0
        FRONT_CONE_WIDTH = 0.3

        side_wall = (
            (ys * self.SIDE > 0) &                   # Correct Side
            (xs > MIN_FORWARD_DIST) &                # Not behind
            (xs < MAX_FORWARD_DIST) &                # Not too far ahead
            (np.abs(ys) < MAX_SIDE_DIST)             # Within range laterally
        )

        front_cone = (
            (ys * self.SIDE < 0) &
            (xs > MIN_FORWARD_DIST) &
            (xs < FRONT_CONE_DIST) &
            (np.abs(ys) < FRONT_CONE_WIDTH)
        )

        valid_indices = side_wall | front_cone
        return xs[valid_indices], ys[valid_indices]


    def get_wall_regression(self, x, y):
        """
        Perform linear regression (least squares) on the points.
        Returns: m (slope), b (intercept) of the line y = mx + b
        """
        if len(x) < 5:
            return None, None

        # It returns coefficients [m, b]
        m, b = np.polyfit(x, y, 1)

        return m, b


    def PID_controller(self, error):
        """
        Creates a PID controller for the car to follow a desired a distance from the wall
        """
        Kp = 5.0
        Ki = 0.1
        Kd = 0.1
        propotional = Kp * error
        self.error_sum = np.clip(self.error_sum + error, -10.0, 10.0)
        integral = Ki * self.error_sum
        derivative = Kd * (error - self.prev_error)
        self.prev_error = error
        return propotional + integral + derivative


    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages.
        """
        # 1. Convert to Cartesian
        xs, ys = self.convert_lidar_cartesian(msg)

        # # 2. Filter for wall points
        wall_xs, wall_ys = self.filter_points(xs, ys)

        # # 3. Regression
        m, b = self.get_wall_regression(wall_xs, wall_ys)
        # when m is empty you don't want car to crash out
        if m is None:
            return
        # Creates a line to visualize the wall
        viz_x = np.linspace(np.min(wall_xs), np.max(wall_xs), num=20)
        viz_y = m * viz_x + b
        VisualizationTools.plot_line(viz_x, viz_y, self.wall_pub, frame="/laser")

        # determines distance from wall
        current_distance = np.abs(b) / np.sqrt(m**2 + 1)
        #current_distance = np.min(np.abs(wall_ys))

        # calculates the error
        error = self.DESIRED_DISTANCE - current_distance + 0.08
        #self.get_logger().info(str(error))

        # PID Controller output and converts that to steering angle
        pid_output = self.PID_controller(error)
        MAX_STEERING_ANGLE = 0.4
        steering_angle = np.clip(-self.SIDE * pid_output, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

        # 4. Drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header = msg.header
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)


    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!

        This is used by the test cases to modify the parameters during testing.
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()