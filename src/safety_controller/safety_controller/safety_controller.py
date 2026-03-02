#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        # Topics (parameterized so we can swap between sim and real car)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_input_topic", "/drive_input")
        self.declare_parameter("drive_output_topic", "/drive")

        # Safety tuning knobs
        self.declare_parameter("safety_angle", 0.35)       # half-width of forward cone (rad)
        self.declare_parameter("hard_stop_distance", 0.3)   # absolute min range before stop (m)
        self.declare_parameter("ttc_threshold", 0.8)        # time-to-collision cutoff (s)

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        drive_in = self.get_parameter("drive_input_topic").get_parameter_value().string_value
        drive_out = self.get_parameter("drive_output_topic").get_parameter_value().string_value
        self.safety_angle = self.get_parameter("safety_angle").get_parameter_value().double_value
        self.hard_stop_dist = self.get_parameter("hard_stop_distance").get_parameter_value().double_value
        self.ttc_threshold = self.get_parameter("ttc_threshold").get_parameter_value().double_value

        # Subscribe to lidar and incoming drive commands, publish safe commands out
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_subscription(AckermannDriveStamped, drive_in, self.drive_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_out, 10)

        # Keep track of what the nav node wants to do
        self.last_speed = 0.0
        self.last_steering = 0.0
        self.last_cmd = AckermannDriveStamped()
        self.safe = True
        self.log_counter = 0

        self.get_logger().info(
            f"Safety controller running: {drive_in} -> {drive_out}, "
            f"cone={np.degrees(self.safety_angle):.0f}deg, "
            f"stop={self.hard_stop_dist}m, ttc={self.ttc_threshold}s")

    def drive_callback(self, msg):
        """Nav command comes in -- forward it only if we think the path is clear."""
        self.last_cmd = msg
        self.last_speed = msg.drive.speed
        self.last_steering = msg.drive.steering_angle

        if self.safe:
            self.drive_pub.publish(msg)

    def compute_ttc(self, ranges, angles, speed):
        """
        TTC = range / (speed * cos(angle))

        cos(angle) projects the range onto the car's forward axis.
        Beams pointing sideways (cos ~ 0) get infinite TTC since
        we're not moving toward them.
        """
        cos_angles = np.cos(angles)
        forward_enough = cos_angles > 0.01
        ttc = np.where(forward_enough, ranges / (speed * cos_angles), np.inf)
        return ttc

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Throw out garbage readings (NaN, zero, or basically touching the sensor)
        bad = ~np.isfinite(ranges) | (ranges < 0.05)
        ranges[bad] = np.inf

        speed = self.last_speed
        steering = self.last_steering
        danger = False

        # === FORWARD CHECK ===
        # Only look at beams in a narrow cone ahead of the car.
        # Shift the cone slightly toward wherever we're steering.
        cone_center = steering * 0.5
        in_cone = ((angles > cone_center - self.safety_angle) &
                   (angles < cone_center + self.safety_angle))

        cone_ranges = ranges[in_cone]
        cone_angles = angles[in_cone]

        if len(cone_ranges) > 0:
            closest = float(np.min(cone_ranges))
        else:
            closest = float("inf")

        # Only worry about forward obstacles if we're actually moving forward
        if speed > 0.01 and closest < float("inf"):
            # Layer 1: something RIGHT in front of us -- just stop
            if closest < self.hard_stop_dist:
                danger = True

            # Layer 2: check time to collision
            if not danger:
                ttc = self.compute_ttc(cone_ranges, cone_angles, speed)
                min_ttc = float(np.min(ttc))
                if min_ttc < self.ttc_threshold:
                    danger = True

        # === REVERSE CHECK ===
        # The lidar doesn't see directly behind, but it covers ~270 degrees
        # so we can check the beams past +/-90 degrees for rearward obstacles
        if not danger and speed < -0.01:
            behind = np.abs(angles) > (np.pi / 2 + self.safety_angle)
            rear_ranges = ranges[behind]
            rear_angles = angles[behind]

            if len(rear_ranges) > 0:
                rear_closest = float(np.min(rear_ranges))

                if rear_closest < self.hard_stop_dist:
                    danger = True
                else:
                    # For rear beams cos is negative, so flip sign
                    rear_cos = -np.cos(rear_angles)
                    rear_forward = rear_cos > 0.01
                    rear_ttc = np.where(rear_forward,
                                        rear_ranges / (abs(speed) * rear_cos),
                                        np.inf)
                    if float(np.min(rear_ttc)) < self.ttc_threshold:
                        danger = True

        # === PUBLISH ===
        self.safe = not danger

        if danger:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(stop_msg)

        # Print status roughly once per second so we can debug
        self.log_counter += 1
        if self.log_counter % 50 == 0:
            tag = "STOP" if danger else "OK"
            self.get_logger().info(
                f"[{tag}] speed={speed:.2f} closest={closest:.2f}m")


def main():
    rclpy.init()
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
