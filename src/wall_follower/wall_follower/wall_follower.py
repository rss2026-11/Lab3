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
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS!
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/input/navigation")
        self.declare_parameter("side", -1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 0.75)

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
        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

        self.subscriber = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)
        self.subscriber

        self.line_vis = self.create_publisher(Marker, '/wall_line', 1)

        self.front_line_vis = self.create_publisher(Marker, '/front_line', 1)

        self.kp = 3.5
        self.kd = 2.2
        # self.ki = 1.0

        self.alpha = 0.3

        self.publish_rate = 0.05
        self.timer = self.create_timer(self.publish_rate, self.pd_controller_callback)
        self.i = 0

        self.front_distance = 0.0
        self.distance = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0

        self.error_sum = 0.0

        # self.get_logger().info('SIDE chosen: "%s"' % self.SIDE)

    def front_scan(self, ranges, angles):
        # Front check
        if (self.SIDE == -1):
            min_front_angle = -np.pi/50
            min_front_index = np.argmin(np.abs(angles - min_front_angle))
            max_front_angle = np.pi/50
            max_front_index = np.argmin(np.abs(angles - max_front_angle))
        else:
            min_front_angle = -np.pi/50
            min_front_index = np.argmin(np.abs(angles - min_front_angle))
            max_front_angle = np.pi/50
            max_front_index = np.argmin(np.abs(angles - max_front_angle))

        front_angles = angles[min_front_index:max_front_index]
        front_ranges = ranges[min_front_index:max_front_index]

        # self.get_logger().info('Distances found "%s"' % front_ranges)

        # dist_filt = front_ranges < 15.0

        # self.get_logger().info('Distance indixes found: "%s"' % dist_filt)

        # filt_front_angles = front_angles[dist_filt]
        # filt_front_ranges = front_ranges[dist_filt]
        filt_front_angles = []
        filt_front_ranges = []
        for i in range(len(front_ranges)):
            if front_ranges[i] > 0.12 and front_ranges[i] < 15.0:
                filt_front_ranges.append(front_ranges[i])
                filt_front_angles.append(front_angles[i])

        if len(filt_front_angles) > 0:
            front_x_values = filt_front_ranges * np.cos(filt_front_angles)
            front_y_values = filt_front_ranges * np.sin(filt_front_angles)
        else:
            front_x_values = [0.0]
            front_y_values = [1.5]

        return front_x_values, front_y_values

    def side_scan(self, ranges, angles):
        # Side Check
        if (self.SIDE == -1):
            min_filter_angle = -(np.pi*10)/19
            min_filter_index = np.argmin(np.abs(angles - min_filter_angle))
            max_filter_angle = -np.pi/4
            max_filter_index = np.argmin(np.abs(angles - max_filter_angle))
        else:
            min_filter_angle = np.pi/6
            min_filter_index = np.argmin(np.abs(angles - min_filter_angle))
            max_filter_angle = (np.pi*10)/19
            max_filter_index = np.argmin(np.abs(angles - max_filter_angle))

        side_angles = angles[min_filter_index:max_filter_index]
        side_ranges = ranges[min_filter_index:max_filter_index]

        # dist_filt = side_ranges < 10.0

        # self.get_logger().info('Distance indixes found: "%s"' % dist_filt)

        # filtered_ranges = side_ranges[dist_filt]
        # filtered_angles = side_angles[dist_filt]

        x_values = side_ranges * np.cos(side_angles)
        y_values = side_ranges * np.sin(side_angles)
        return x_values, y_values

    def distance_calc(self, x_values, y_values, front=False):
        coefficients = np.polyfit(x_values, y_values, 1)

        raw_distance = np.abs(coefficients[1]) / np.sqrt(coefficients[0]**2 + 1)

        if front:
            self.get_logger().info(f'FD found: {raw_distance}')
            if self.front_distance == 0.0:
                self.front_distance = raw_distance # Initialize on first run
            else:
                self.front_distance = (self.alpha * raw_distance) + ((1.0 - self.alpha) * self.front_distance)
                # self.front_distance = raw_distance
        else:
            if self.distance == 0.0:
                self.distance = raw_distance
            else:
                self.distance = (self.alpha * raw_distance) + ((1.0 - self.alpha) * self.distance)
                # self.distance = raw_distance


    # TODO: Write your callback functions here
    def listener_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # Front calculation
        front_x_values, front_y_values = self.front_scan(ranges, angles)

        self.distance_calc(front_x_values, front_y_values, True)
        VisualizationTools.plot_line(front_x_values, front_y_values, self.front_line_vis)

        # Side calculation
        x_values, y_values = self.side_scan(ranges, angles)
        VisualizationTools.plot_line(x_values, y_values, self.line_vis)

        self.distance_calc(x_values, y_values)

    def pd_controller_callback(self):

        error = self.DESIRED_DISTANCE - self.distance
        self.get_logger().info('Distance found: "%s"' % self.distance)
        # self.get_logger().info('FD found: "%s"' % self.front_distance)
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.prev_time) / 1e9
        d_error = (error - self.prev_error)/dt
        d_error = np.clip(d_error, -2.5, 2.5)
        self.error_sum = self.error_sum+error
        self.error_sum = np.clip(self.error_sum, -10.0, 10.0)
        control_signal = error * self.kp + self.kd*(d_error)
        # control_signal = error * self.kp + self.kd*(d_error) + self.ki*(self.error_sum)
        self.prev_error = error
        self.prev_time = now.nanoseconds
        if self.front_distance <= (1.1):
            # self.get_logger().info("Front")
            steer_angle = 4.0 * -(self.SIDE)
            speed = min(1.8, self.VELOCITY * 0.8)
        else:
            # self.get_logger().info("Normal steering")
            steer_angle = control_signal * -(self.SIDE)
            # self.get_logger().info("")
            speed = self.VELOCITY

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = now.to_msg()
        drive_msg.drive.steering_angle = steer_angle

        drive_msg.drive.steering_angle_velocity = 0.0

        drive_msg.drive.speed = speed
        self.publisher_.publish(drive_msg)

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
