import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyControllerTester(Node):

    def __init__(self):
        super().__init__("safety_controller_tester")

        self.pub = self.create_publisher(
            AckermannDriveStamped,"/vesc/high_level/input/nav_0",10)

        self.create_timer(0.05, self.publish_drive)  # 20Hz
        self.get_logger().info("Driving forward at 1.0 m/s — safety controller will handle stopping")


    def publish_drive(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 3.0
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyControllerTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
