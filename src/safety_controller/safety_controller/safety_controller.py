import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        # 1. Parameters
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("incoming_cmd_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("safety_output_topic", "/vesc/low_level/input/safety")
        self.declare_parameter("stop_distance", 0.2)
        self.declare_parameter("half_cone", math.pi / 3)
        self.declare_parameter("a_max", 4.0)

        scan_topic          = self.get_parameter("scan_topic").value
        incoming_cmd_topic  = self.get_parameter("incoming_cmd_topic").value
        safety_output_topic = self.get_parameter("safety_output_topic").value
        self.stop_distance  = self.get_parameter("stop_distance").value
        self.half_cone      = self.get_parameter("half_cone").value
        self.a_max          = self.get_parameter("a_max").value

        # 2. Subscribers
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_subscription(AckermannDriveStamped, incoming_cmd_topic, self.drive_callback, 10)

        # 3. Publisher
        self.pub = self.create_publisher(AckermannDriveStamped, safety_output_topic, 10)

        # 4. State
        self.current_speed    = 0.0
        self.current_steering = 0.0


    def drive_callback(self, msg):
        self.current_speed    = msg.drive.speed
        self.current_steering = msg.drive.steering_angle


    def scan_callback(self, msg):

        # Step 1 — if not moving, nothing to do
        if abs(self.current_speed) < 0.05:
            return

        # Step 2 — extract ranges and angles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Step 3 — clean bad readings
        bad = ~np.isfinite(ranges) | (ranges < 0.05)
        ranges[bad] = np.inf

        # Step 4 — filter to forward cone only
        in_cone = (angles > -self.half_cone) & (angles < self.half_cone)
        cone_ranges = ranges[in_cone]
        cone_angles = angles[in_cone]

        # Step 5 — find closest obstacle in cone
        closest = np.min(cone_ranges)
        closest_angle_deg = math.degrees(cone_angles[np.argmin(cone_ranges)])

       ## self.get_logger().info(f"Closest obstacle: {closest:.3f}m at {closest_angle_deg:.1f}deg")

        # Step 6 — Layer 1: hard stop
        if closest < self.stop_distance:
         ##   self.get_logger().warn(f"[HARD STOP] Obstacle at {closest:.3f}m / {closest_angle_deg:.1f}deg — publishing STOP")
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            self.pub.publish(stop_msg)
            return

        # Step 7 — Layer 2: gradual braking
        # brake_distance = (self.current_speed ** 2) / (2 * self.a_max)
           
        m = 2.5 / 0.3
        b = 0.5 - m * (0.5) 
        
        # calcualting brake distance
        brake_distance = (self.current_speed - b) / m

        if closest - self.stop_distance < brake_distance:
            safe_speed = self.current_speed * (closest - self.stop_distance) / brake_distance
           ## self.get_logger().warn(f"[BRAKING] {closest:.3f}m ahead — speed {self.current_speed:.3f} -> {safe_speed:.3f} m/s")
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = safe_speed
            brake_msg.drive.steering_angle = 0.0
            self.pub.publish(brake_msg)


def main():
    rclpy.init()
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()





# #!/usr/bin/env python3
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from ackermann_msgs.msg import AckermannDriveStamped


# class SafetyController(Node):

#     def __init__(self):
#         super().__init__("safety_controller")

#         # Topics (parameterized so we can swap between sim and real car)
#         self.declare_parameter("scan_topic", "/scan")
#         self.declare_parameter("drive_input_topic", "/drive_input")
#         self.declare_parameter("drive_output_topic", "/drive")

#         # Safety tuning knobs
#         self.declare_parameter("safety_angle", 0.35)       # half-width of forward cone (rad)
#         self.declare_parameter("hard_stop_distance", 0.3)   # absolute min range before stop (m)
#         self.declare_parameter("ttc_threshold", 0.8)        # time-to-collision cutoff (s)

#         scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
#         drive_in = self.get_parameter("drive_input_topic").get_parameter_value().string_value
#         drive_out = self.get_parameter("drive_output_topic").get_parameter_value().string_value
#         self.safety_angle = self.get_parameter("safety_angle").get_parameter_value().double_value
#         self.hard_stop_dist = self.get_parameter("hard_stop_distance").get_parameter_value().double_value
#         self.ttc_threshold = self.get_parameter("ttc_threshold").get_parameter_value().double_value

#         # Subscribe to lidar and incoming drive commands, publish safe commands out
#         self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
#         self.create_subscription(AckermannDriveStamped, drive_in, self.drive_callback, 10)
#         self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_out, 10)

#         # Keep track of what the nav node wants to do
#         self.last_speed = 0.0
#         self.last_steering = 0.0
#         self.last_cmd = AckermannDriveStamped()
#         self.safe = True
#         self.log_counter = 0

#         self.get_logger().info(
#             f"Safety controller running: {drive_in} -> {drive_out}, "
#             f"cone={np.degrees(self.safety_angle):.0f}deg, "
#             f"stop={self.hard_stop_dist}m, ttc={self.ttc_threshold}s")

#     def drive_callback(self, msg):
#         """Nav command comes in -- forward it only if we think the path is clear."""
#         self.last_cmd = msg
#         self.last_speed = msg.drive.speed
#         self.last_steering = msg.drive.steering_angle

#         if self.safe:
#             self.drive_pub.publish(msg)

#     def compute_ttc(self, ranges, angles, speed):
#         """
#         TTC = range / (speed * cos(angle))

#         cos(angle) projects the range onto the car's forward axis.
#         Beams pointing sideways (cos ~ 0) get infinite TTC since
#         we're not moving toward them.
#         """
#         cos_angles = np.cos(angles)
#         forward_enough = cos_angles > 0.01
#         ttc = np.where(forward_enough, ranges / (speed * cos_angles), np.inf)
#         return ttc

#     def scan_callback(self, msg):
#         ranges = np.array(msg.ranges, dtype=np.float64)
#         angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

#         # Throw out garbage readings (NaN, zero, or basically touching the sensor)
#         bad = ~np.isfinite(ranges) | (ranges < 0.05)
#         ranges[bad] = np.inf

#         speed = self.last_speed
#         steering = self.last_steering
#         danger = False

#         # === FORWARD CHECK ===
#         # Only look at beams in a narrow cone ahead of the car.
#         # Shift the cone slightly toward wherever we're steering.
#         cone_center = steering * 0.5
#         in_cone = ((angles > cone_center - self.safety_angle) &
#                    (angles < cone_center + self.safety_angle))

#         cone_ranges = ranges[in_cone]
#         cone_angles = angles[in_cone]

#         if len(cone_ranges) > 0:
#             closest = float(np.min(cone_ranges))
#         else:
#             closest = float("inf")

#         # Only worry about forward obstacles if we're actually moving forward
#         if speed > 0.01 and closest < float("inf"):
#             # Layer 1: something RIGHT in front of us -- just stop
#             if closest < self.hard_stop_dist:
#                 danger = True

#             # Layer 2: check time to collision
#             if not danger:
#                 ttc = self.compute_ttc(cone_ranges, cone_angles, speed)
#                 min_ttc = float(np.min(ttc))
#                 if min_ttc < self.ttc_threshold:
#                     danger = True

#         # The lidar doesn't see directly behind, but it covers ~270 degrees
#         # so we can check the beams past +/-90 degrees for rearward obstacles
#         if not danger and speed < -0.01:
#             behind = np.abs(angles) > (np.pi / 2 + self.safety_angle)
#             rear_ranges = ranges[behind]
#             rear_angles = angles[behind]

#             if len(rear_ranges) > 0:
#                 rear_closest = float(np.min(rear_ranges))

#                 if rear_closest < self.hard_stop_dist:
#                     danger = True
#                 else:
#                     # For rear beams cos is negative, so flip sign
#                     rear_cos = -np.cos(rear_angles)
#                     rear_forward = rear_cos > 0.01
#                     rear_ttc = np.where(rear_forward,
#                                         rear_ranges / (abs(speed) * rear_cos),
#                                         np.inf)
#                     if float(np.min(rear_ttc)) < self.ttc_threshold:
#                         danger = True

#         # === PUBLISH ===
#         self.safe = not danger

#         if danger:
#             stop_msg = AckermannDriveStamped()
#             stop_msg.drive.speed = 0.0
#             stop_msg.drive.steering_angle = 0.0
#             self.drive_pub.publish(stop_msg)

#         # Print status roughly once per second so we can debug
#         self.log_counter += 1
#         if self.log_counter % 50 == 0:
#             tag = "STOP" if danger else "OK"
#             self.get_logger().info(
#                 f"[{tag}] speed={speed:.2f} closest={closest:.2f}m")


# def main():
#     rclpy.init()
#     node = SafetyController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
