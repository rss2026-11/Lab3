import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


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
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(String, "/part_b/state", self.state_callback, 10)

        # 3. Publisher
        self.pub = self.create_publisher(AckermannDriveStamped, safety_output_topic, 10)

        # 4. State
        self.current_speed    = 0.0
        self.current_steering = 0.0
        self.mission_state    = "INIT"

        # actual pose
        self.actual_speed   = 0.0
        self.last_cmd_speed = 0.0  # from incoming commands
        self.last_cmd_msg   = None

        # Stuck + reverse logic
        self.is_stopped_due_to_obstacle = False
        self.stopped_time_start = None
        self.reverse_active = False
        self.reverse_end_time = None

    def state_callback(self, msg):
        self.mission_state = msg.data

    def drive_callback(self, msg):
        self.current_speed    = msg.drive.speed
        self.current_steering = msg.drive.steering_angle
        self.last_cmd_speed   = msg.drive.speed
        self.last_cmd_msg     = msg

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.actual_speed = math.sqrt(vx * vx + vy * vy)

    def publish_reverse(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = -1.0
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)

    def scan_callback(self, msg):
        now = time.time()

        # Step 2 — extract ranges and angles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Step 3 — clean bad readings
        bad = ~np.isfinite(ranges) | (ranges < 0.05)
        ranges[bad] = np.inf

        # Step 4 — filter to forward or rear cone based on driving direction
        if "BACKUP" in self.mission_state:
            in_cone = (np.abs(angles) > math.pi - self.half_cone)
        elif self.current_speed >= 0:
            cone_center = self.current_steering * 0.5
            in_cone = (angles > cone_center - self.half_cone) & (angles < self.half_cone + cone_center)
        else:
            in_cone = (np.abs(angles) > math.pi - self.half_cone)

        cone_ranges = ranges[in_cone]
        cone_angles = angles[in_cone]

        # Filter out points that are far to the side (lateral distance > 0.4m)
        if len(cone_ranges) > 0:
            lateral_distances = np.abs(cone_ranges * np.sin(cone_angles))
            in_path = lateral_distances < 0.4
            cone_ranges = cone_ranges[in_path]
            cone_angles = cone_angles[in_path]

        # If no obstacle in cone
        if len(cone_ranges) == 0:
            # If we were reversing and obstacle disappeared, we can stop reverse early
            if self.reverse_active:
                if now >= self.reverse_end_time:
                    self.reverse_active = False
                    self.is_stopped_due_to_obstacle = False
                    self.stopped_time_start = None
                    self.get_logger().info("[REVERSE COMPLETE] No obstacle in cone — resuming normal safety control")
                else:
                    # Still within 1s reverse window, keep backing up
                    self.publish_reverse()
                    return
            else:
                # No obstacle, reset stuck state
                self.is_stopped_due_to_obstacle = False
                self.stopped_time_start = None
            return

        closest = float(np.min(cone_ranges))
        closest_angle_deg = math.degrees(cone_angles[np.argmin(cone_ranges)])

        # --- Decide if we should start a reverse maneuver (5s stopped due to obstacle) ---
        if self.is_stopped_due_to_obstacle and self.stopped_time_start is not None and not self.reverse_active:
            if now - self.stopped_time_start >= 5.0:
                self.get_logger().warn("[STUCK] 5 seconds stopped due to obstacle — initiating REVERSE MANEUVER")
                self.reverse_active = True
                self.reverse_end_time = now + 1.0  # reverse for 1 second

        # --- If reverse maneuver is active, override everything else ---
        if self.reverse_active:
            # Back up for up to 1s AND until closest obstacle is at least 0.8m away
            if now < self.reverse_end_time and closest < 0.8:
                self.get_logger().warn(
                    f"[REVERSING] closest obstacle {closest:.3f}m / {closest_angle_deg:.1f}deg — backing up"
                )
                self.publish_reverse()
                return
            else:
                self.reverse_active = False
                self.is_stopped_due_to_obstacle = False
                self.stopped_time_start = None
                self.get_logger().info(
                    f"[REVERSE COMPLETE] closest obstacle {closest:.3f}m — resuming normal safety control"
                )
                # fall through to normal safety logic with updated closest

        # Step 6 — Layer 1: hard stop
        if closest < self.stop_distance:
            self.get_logger().warn(
                f"[HARD STOP] Obstacle at {closest:.3f}m / {closest_angle_deg:.1f}deg — publishing STOP"
            )
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            self.pub.publish(stop_msg)

            # Start or keep stuck timer (for later reverse)
            if not self.is_stopped_due_to_obstacle:
                self.is_stopped_due_to_obstacle = True
                self.stopped_time_start = now

            return

        # Step 7 — Layer 2: gradual braking
        m = 0.1
        b = 0.50
        brake_distance = self.current_speed * m + b

        if closest < brake_distance:
            safe_speed = self.current_speed * (closest) / brake_distance
            self.get_logger().warn(
                f"[BRAKING] {closest:.3f}m ahead — speed {self.current_speed:.3f} -> {safe_speed:.3f} m/s"
            )
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = safe_speed
            brake_msg.drive.steering_angle = 0.0
            self.pub.publish(brake_msg)

            # If braking effectively stops the car, start/maintain stuck timer
            if safe_speed < 0.05:
                if not self.is_stopped_due_to_obstacle:
                    self.is_stopped_due_to_obstacle = True
                    self.stopped_time_start = now

            return

        # If obstacle is far enough, reset stuck state
        if closest > self.stop_distance + 0.1:
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
# import time
# from ackermann_msgs.msg import AckermannDriveStamped
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String




# class SafetyController(Node):

#     def __init__(self):
#         super().__init__("safety_controller")

#         # 1. Parameters
#         self.declare_parameter("scan_topic", "/scan")
#         self.declare_parameter("incoming_cmd_topic", "/vesc/high_level/input/navigation")
#         self.declare_parameter("safety_output_topic", "/vesc/low_level/input/safety")
#         self.declare_parameter("stop_distance", 0.4)
#         # Reduced from math.pi / 3 to avoid seeing walls when following racelines
#         self.declare_parameter("half_cone", 0.42)
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
#         self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

#         # 3. Publisher
#         self.pub = self.create_publisher(AckermannDriveStamped, safety_output_topic, 10)

#         # 4. State
#         self.current_speed    = 0.0
#         self.current_steering = 0.0
#         self.mission_state    = "INIT"

#         # actual pose
#         self.actual_speed = 0.0
#         self.last_cmd_speed = 0.0  # from incoming commands


#         self.create_subscription(String, "/part_b/state", self.state_callback, 10)

#         # NEW: Stuck + reverse logic
#         self.is_stopped_due_to_obstacle = False
#         self.stopped_time_start = None
#         self.reverse_active = False
#         self.reverse_end_time = None

#     def state_callback(self, msg):
#         self.mission_state = msg.data

#     def drive_callback(self, msg):
#         self.current_speed    = msg.drive.speed
#         self.current_steering = msg.drive.steering_angle
#         self.last_cmd_speed   = msg.drive.speed
#         self.last_cmd_msg     = msg  # we’ll use this for pass-through

#     def odom_callback(self, msg):
#         vx = msg.twist.twist.linear.x
#         vy = msg.twist.twist.linear.y
#         self.actual_speed = math.sqrt(vx*vx + vy*vy)


#     # NEW helper: publish reverse command
#     def publish_reverse(self):
#         msg = AckermannDriveStamped()
#         msg.drive.speed = -1.0
#         msg.drive.steering_angle = 0.0
#         self.pub.publish(msg)


#     def scan_callback(self, msg):

#         now = time.time()

#         # NEW: If reverse maneuver is active, override everything
#         if self.reverse_active:
#             if now < self.reverse_end_time:
#                 self.publish_reverse()
#                 return
#             else:
#                 self.reverse_active = False
#                 self.is_stopped_due_to_obstacle = False
#                 self.stopped_time_start = None
#                 self.get_logger().info("[REVERSE COMPLETE] Resuming normal safety control")
#                 return

#         # # NEW: Check if car is stuck (we do this regardless of commanded speed now)
#         # if self.is_stopped_due_to_obstacle and self.stopped_time_start is not None:
#         #     if now - self.stopped_time_start > 5.0:
#         #         self.get_logger().warn("[STUCK] 5 seconds passed — initiating REVERSE MANEUVER")
#         #         self.reverse_active = True
#         #         self.reverse_end_time = now + 1.0

#         # NEW: odometry-based stuck detection
#         if self.is_stopped_due_to_obstacle and self.stopped_time_start is not None:
#             # car is supposed to be moving but isn't
#             commanded_moving = abs(self.last_cmd_speed) > 0.2
#             actually_stopped = self.actual_speed < 0.05

#             if commanded_moving and actually_stopped and (now - self.stopped_time_start > 3.0):
#                 self.get_logger().warn("[STUCK] 3s commanded motion but no odom movement — initiating REVERSE MANEUVER")
#                 self.reverse_active = True
#                 self.reverse_end_time = now + 1.0


#         # Step 2 — extract ranges and angles
#         ranges = np.array(msg.ranges)
#         angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

#         # Step 3 — clean bad readings
#         bad = ~np.isfinite(ranges) | (ranges < 0.05)
#         ranges[bad] = np.inf

#         # Step 4 — filter to forward or rear cone based on driving direction
#         # If the mission is actively trying to backup, FORCE the cone to the rear!
#         if "BACKUP" in self.mission_state:
#             in_cone = (np.abs(angles) > math.pi - self.half_cone)
#         elif self.current_speed >= 0:
#             cone_center = self.current_steering * 0.5
#             in_cone = (angles > cone_center - self.half_cone) & (angles < self.half_cone + cone_center)
#         else:
#             # Reversing manually: check the rear!
#             in_cone = (np.abs(angles) > math.pi - self.half_cone)
#         cone_ranges = ranges[in_cone]
#         cone_angles = angles[in_cone]

#         # Filter out points that are far to the side (lateral distance > 0.4m)
#         # This prevents braking for walls when driving parallel to them.
#         if len(cone_ranges) > 0:
#             lateral_distances = np.abs(cone_ranges * np.sin(cone_angles))
#             in_path = lateral_distances < 0.4
#             cone_ranges = cone_ranges[in_path]
#             cone_angles = cone_angles[in_path]

#         # Step 5 — find closest obstacle in cone
#         if len(cone_ranges) == 0:
#             # NEW: If obstacle disappeared, reset stuck state
#             self.is_stopped_due_to_obstacle = False
#             self.stopped_time_start = None
#             return

#         closest = float(np.min(cone_ranges))
#         closest_angle_deg = math.degrees(cone_angles[np.argmin(cone_ranges)])

#         # Step 6 — Layer 1: hard stop
#         if closest < self.stop_distance:
#             self.get_logger().warn(f"[HARD STOP] Obstacle at {closest:.3f}m / {closest_angle_deg:.1f}deg — publishing STOP")
#             stop_msg = AckermannDriveStamped()
#             stop_msg.drive.speed = 0.0
#             stop_msg.drive.steering_angle = 0.0
#             self.pub.publish(stop_msg)

#             # NEW: Start stuck timer
#             if not self.is_stopped_due_to_obstacle:
#                 self.is_stopped_due_to_obstacle = True
#                 self.stopped_time_start = now

#             return

#         # Step 7 — Layer 2: gradual braking  # oldest line
#         # brake_distance = (self.current_speed ** 2) / (2 * self.a_max)  # oldest line

#         # Linear break function adatpting to speed
#         m = 0.1
#         b = 0.50
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

#             # NEW: If braking brings car to near stop, start stuck timer
#             if safe_speed < 0.05:
#                 if not self.is_stopped_due_to_obstacle:
#                     self.is_stopped_due_to_obstacle = True
#                     self.stopped_time_start = now

#             return

#         # NEW: If obstacle is gone (with hysteresis to prevent sensor jitter resets), reset stuck state
#         if closest > self.stop_distance + 0.1:
#             self.is_stopped_due_to_obstacle = False
#             self.stopped_time_start = None


# def main():
#     rclpy.init()
#     node = SafetyController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()



