# Testnode ob die Daten der 'CameraNode' richtig gepublisht / subscribt werden
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, Bool, Int32
from canalchecker_interface.msg import ArucoDetection

class TestNode(Node):
    def __init__(self):
        super().__init__('testnode')

        self.sub1 = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.listener_callback_fnc1,
            10
        )

    def listener_callback_fnc1(self, msg: ArucoDetection):
        self.get_logger().info(f'Distance: {msg.aruco_distance}\nAngle: {msg.aruco_angle}\nID: {msg.aruco_id}')


def main():
    rclpy.init()
    try:
        test_node = TestNode()
        rclpy.spin(test_node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()