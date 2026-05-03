import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        # 1. Parameters
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("incoming_cmd_topic", "/vesc/high_level/input/navigation")
        self.declare_parameter("safety_output_topic", "/vesc/low_level/input/safety")
        self.declare_parameter("stop_distance", 0.4)
        # Reduced from math.pi / 3 to avoid seeing walls when following racelines
        self.declare_parameter("half_cone", 0.42)
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

        # NEW: Stuck + reverse logic
        self.is_stopped_due_to_obstacle = False
        self.stopped_time_start = None
        self.reverse_active = False
        self.reverse_end_time = None


    def drive_callback(self, msg):
        self.current_speed    = msg.drive.speed
        self.current_steering = msg.drive.steering_angle


    # NEW helper: publish reverse command
    def publish_reverse(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = -0.5
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)


    def scan_callback(self, msg):

        now = time.time()

        # NEW: If reverse maneuver is active, override everything
        if self.reverse_active:
            if now < self.reverse_end_time:
                self.publish_reverse()
                return
            else:
                self.reverse_active = False
                self.get_logger().info("[REVERSE COMPLETE] Resuming normal safety control")
                return

        # Step 1 — if not moving, nothing to do
        if abs(self.current_speed) < 0.05:
            # NEW: If car is not moving but was previously stopped due to obstacle, check timer
            if self.is_stopped_due_to_obstacle and self.stopped_time_start is not None:
                if now - self.stopped_time_start > 5.0:
                    self.get_logger().warn("[STUCK] 5 seconds passed — initiating REVERSE MANEUVER")
                    self.reverse_active = True
                    self.reverse_end_time = now + 1.0
            return

        # Step 2 — extract ranges and angles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Step 3 — clean bad readings
        bad = ~np.isfinite(ranges) | (ranges < 0.05)
        ranges[bad] = np.inf

        # Step 4 — filter to forward cone only
        # Shift the cone slightly toward wherever we're steering.
        cone_center = self.current_steering * 0.5
        in_cone = (angles > cone_center - self.half_cone) & (angles < cone_center + self.half_cone)
        cone_ranges = ranges[in_cone]
        cone_angles = angles[in_cone]

        # Filter out points that are far to the side (lateral distance > 0.4m)
        # This prevents braking for walls when driving parallel to them.
        if len(cone_ranges) > 0:
            lateral_distances = np.abs(cone_ranges * np.sin(cone_angles))
            in_path = lateral_distances < 0.4
            cone_ranges = cone_ranges[in_path]
            cone_angles = cone_angles[in_path]

        # Step 5 — find closest obstacle in cone
        if len(cone_ranges) == 0:
            # NEW: If obstacle disappeared, reset stuck state
            self.is_stopped_due_to_obstacle = False
            self.stopped_time_start = None
            return

        closest = float(np.min(cone_ranges))
        closest_angle_deg = math.degrees(cone_angles[np.argmin(cone_ranges)])

        # Step 6 — Layer 1: hard stop
        if closest < self.stop_distance:
            self.get_logger().warn(f"[HARD STOP] Obstacle at {closest:.3f}m / {closest_angle_deg:.1f}deg — publishing STOP")
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            self.pub.publish(stop_msg)

            # NEW: Start stuck timer
            if not self.is_stopped_due_to_obstacle:
                self.is_stopped_due_to_obstacle = True
                self.stopped_time_start = now

            # NEW: Check if stuck for 5 seconds
            if now - self.stopped_time_start > 5.0:
                self.get_logger().warn("[STUCK] 5 seconds passed — initiating REVERSE MANEUVER")
                self.reverse_active = True
                self.reverse_end_time = now + 1.0

            return

        # Step 7 — Layer 2: gradual braking  # oldest line
        # brake_distance = (self.current_speed ** 2) / (2 * self.a_max)  # oldest line

        # Linear break function adatpting to speed
        m = 0.1
        b = 0.35
        brake_distance = self.current_speed * m + b

        # if closest - self.stop_distance < brake_distance: # old line
        if closest < brake_distance:
            # safe_speed = self.current_speed * (closest - self.stop_distance) / brake_distance  # old line
            safe_speed = self.current_speed * (closest) / brake_distance
            self.get_logger().warn(f"[BRAKING] {closest:.3f}m ahead — speed {self.current_speed:.3f} -> {safe_speed:.3f} m/s")
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = safe_speed
            brake_msg.drive.steering_angle = 0.0
            self.pub.publish(brake_msg)

            # NEW: If braking brings car to near stop, start stuck timer
            if safe_speed < 0.05:
                if not self.is_stopped_due_to_obstacle:
                    self.is_stopped_due_to_obstacle = True
                    self.stopped_time_start = now

                if now - self.stopped_time_start > 5.0:
                    self.get_logger().warn("[STUCK] 5 seconds passed — initiating REVERSE MANEUVER")
                    self.reverse_active = True
                    self.reverse_end_time = now + 1.0

            return

        # NEW: If obstacle is gone, reset stuck state
        self.is_stopped_due_to_obstacle = False
        self.stopped_time_start = None


def main():
    rclpy.init()
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



# import math
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from ackermann_msgs.msg import AckermannDriveStamped


# class SafetyController(Node):

#     def __init__(self):
#         super().__init__("safety_controller")

#         # 1. Parameters
#         self.declare_parameter("scan_topic", "/scan")
#         self.declare_parameter("incoming_cmd_topic", "/vesc/input/navigation")
#         self.declare_parameter("safety_output_topic", "/vesc/low_level/input/safety")
#         self.declare_parameter("stop_distance", 0.2)
#         # Reduced from math.pi / 3 to avoid seeing walls when following racelines
#         self.declare_parameter("half_cone", 0.35)
#         self.declare_parameter("a_max", 4.0)

#         scan_topic          = self.get_parameter("scan_topic").value
#         incoming_cmd_topic  = self.get_parameter("incoming_cmd_topic").value
#         safety_output_topic = self.get_parameter("safety_output_topic").value
#         self.stop_distance  = self.get_parameter("stop_distance").value
#         self.half_cone      = self.get_parameter("half_cone").value
#         self.a_max          = self.get_parameter("a_max").value

#         # 2. Subscribers
#         self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
#         self.create_subscription(AckermannDriveStamped, incoming_cmd_topic, self.drive_callback, 10)

#         # 3. Publisher
#         self.pub = self.create_publisher(AckermannDriveStamped, safety_output_topic, 10)

#         # 4. State
#         self.current_speed    = 0.0
#         self.current_steering = 0.0


#     def drive_callback(self, msg):
#         self.current_speed    = msg.drive.speed
#         self.current_steering = msg.drive.steering_angle


#     def scan_callback(self, msg):

#         # Step 1 — if not moving, nothing to do
#         if abs(self.current_speed) < 0.05:
#             return

#         # Step 2 — extract ranges and angles
#         ranges = np.array(msg.ranges)
#         angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

#         # Step 3 — clean bad readings
#         bad = ~np.isfinite(ranges) | (ranges < 0.05)
#         ranges[bad] = np.inf

#         # Step 4 — filter to forward cone only
#         # Shift the cone slightly toward wherever we're steering.
#         cone_center = self.current_steering * 0.5
#         in_cone = (angles > cone_center - self.half_cone) & (angles < cone_center + self.half_cone)
#         cone_ranges = ranges[in_cone]
#         cone_angles = angles[in_cone]

#         # Filter out points that are far to the side (lateral distance > 0.35m)
#         # This prevents braking for walls when driving parallel to them.
#         if len(cone_ranges) > 0:
#             lateral_distances = np.abs(cone_ranges * np.sin(cone_angles))
#             in_path = lateral_distances < 0.35
#             cone_ranges = cone_ranges[in_path]
#             cone_angles = cone_angles[in_path]

#         # Step 5 — find closest obstacle in cone
#         if len(cone_ranges) == 0:
#             return

#         closest = float(np.min(cone_ranges))
#         closest_angle_deg = math.degrees(cone_angles[np.argmin(cone_ranges)])

#         # self.get_logger().info(f"Closest obstacle: {closest:.3f}m at {closest_angle_deg:.1f}deg")

#         # Step 6 — Layer 1: hard stop
#         if closest < self.stop_distance:
#             self.get_logger().warn(f"[HARD STOP] Obstacle at {closest:.3f}m / {closest_angle_deg:.1f}deg — publishing STOP")
#             stop_msg = AckermannDriveStamped()
#             stop_msg.drive.speed =  -0.1 # 0.0
#             stop_msg.drive.steering_angle = 0.0
#             self.pub.publish(stop_msg)
#             return

#         # Step 7 — Layer 2: gradual braking  # oldest line
#         # brake_distance = (self.current_speed ** 2) / (2 * self.a_max)  # oldest line

#         # Linear break function adatpting to speed
#         m = 0.1
#         b = 0.35
#         brake_distance = self.current_speed * m + b

#         # if closest - self.stop_distance < brake_distance: # old line
#         if closest < brake_distance:
#             # safe_speed = self.current_speed * (closest - self.stop_distance) / brake_distance  # old line
#             safe_speed = self.current_speed * (closest) / brake_distance
#             self.get_logger().warn(f"[BRAKING] {closest:.3f}m ahead — speed {self.current_speed:.3f} -> {safe_speed:.3f} m/s")
#             brake_msg = AckermannDriveStamped()
#             brake_msg.drive.speed = safe_speed
#             brake_msg.drive.steering_angle = 0.0
#             self.pub.publish(brake_msg)


# def main():
#     rclpy.init()
#     node = SafetyController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
