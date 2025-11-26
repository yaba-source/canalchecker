# Testnode ob die Daten der 'CameraNode' richtig gepublisht / subscribt werden
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, Bool

class TestNode(Node):
    def __init__(self):
        super().__init__('testnode')

        self.sub1 = self.create_subscription(
            UInt32,
            '/aruco_distance',
            self.listener_callback_fnc1,
            10
        )

        self.sub1 = self.create_subscription(
            Bool,
            '/aruco_detected',
            self.listener_callback_fnc2,
            10
        )

        self.sub1 = self.create_subscription(
            UInt32,
            '/aruco_angle',
            self.listener_callback_fnc3,
            10
        )

        self.sub1 = self.create_subscription(
            Bool,
            '/aruco_id',
            self.listener_callback_fnc4,
            10
        )

    def listener_callback_fnc1(self, msg: UInt32):
        self.get_logger().info(f'Distance: {msg.data}')

    def listener_callback_fnc2(self, msg: Bool):
        self.get_logger().info(f'Detected: {msg.data}')

    def listener_callback_fnc3(self, msg: UInt32):
        self.get_logger().info(f'angle: {msg.data}')

    def listener_callback_fnc4(self, msg: Bool):
        self.get_logger().info(f'ID: {msg.data}')


def main():
    rclpy.init()
    try:
        test_node = TestNode()
        rclpy.spin(test_node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()