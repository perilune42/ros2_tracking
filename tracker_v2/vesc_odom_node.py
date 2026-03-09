"""
Publishes the calibrated motor RPM read back from the VESC at 30 Hz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from .vesc_submodule.vesc_client import VESC_

NODE_NAME = 'vesc_odom_node'
PUBLISH_TOPIC = '/vesc_rpm'


class VescOdom(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.vesc = VESC_()
        self.motor_rpm_pub = self.create_publisher(Int32, PUBLISH_TOPIC, 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('K_rpm', 1.5),
                ('K_rpm_offset', -125.88),
                ('K_v', 4921.82),
                ('K_v_offset', 93.59),
            ],
        )
        self.K_rpm = float(self.get_parameter('K_rpm').value)
        self.K_rpm_offset = float(self.get_parameter('K_rpm_offset').value)

        self.create_timer(1.0 / 30.0, self._publish_rpm)
        self.get_logger().info(f'{NODE_NAME} started')

    def _publish_rpm(self):
        try:
            raw_rpm = int(self.vesc.get_rpm())
            est_rpm = int(self.K_rpm * raw_rpm + self.K_rpm_offset)
            msg = Int32()
            msg.data = est_rpm
            self.motor_rpm_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'VESC RPM read failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VescOdom()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
