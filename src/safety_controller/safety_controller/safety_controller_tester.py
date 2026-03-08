import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import time


class SafetyControllerTester(Node):

    def __init__(self):
        super().__init__("safety_controller_tester")

        self.pub = self.create_publisher(
            AckermannDriveStamped,
            "/vesc/high_level/input/nav_0",
            10
        )

        # Test sequence: (duration_sec, speed, steering_angle)
        self.tests = [
            (3.0, 0.5, 0.0),    # slow forward
            (3.0, 1.0, 0.0),    # normal forward
            (3.0, 2.0, 0.0),    # fast forward
            (3.0, -1.0, 0.0),   # reverse
            (3.0, 1.0, 0.4),    # forward with steering
            (3.0, 1.0, -0.4),   # forward with opposite steering
            (3.0, 0.0, 0.0),    # stop command
        ]

        self.current_test_index = 0
        self.test_start_time = time.time()

        self.create_timer(0.05, self.run_test)  # 20 Hz
        self.get_logger().info("Starting multi-case safety controller test sequence.")


    def run_test(self):
        if self.current_test_index >= len(self.tests):
            self.get_logger().info("All tests completed.")
            return

        duration, speed, angle = self.tests[self.current_test_index]

        # Publish command
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub.publish(msg)

        # Log occasionally
        if int(time.time() * 10) % 20 == 0:
            self.get_logger().info(
                f"Test {self.current_test_index+1}/{len(self.tests)}: "
                f"speed={speed}, angle={angle}"
            )

        # Move to next test when time is up
        if time.time() - self.test_start_time > duration:
            self.current_test_index += 1
            self.test_start_time = time.time()
            self.get_logger().info("Switching to next test case.")


def main():
    rclpy.init()
    node = SafetyControllerTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
