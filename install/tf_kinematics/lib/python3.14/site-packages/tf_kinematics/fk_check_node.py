import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class FKCheckNode(Node):
    def __init__(self):
        super().__init__('fk_check_node')

        self.declare_parameter('child_frame', 'left_ankle_pitch_link')
        self.declare_parameter('tolerance_m', 0.04)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.latest_marker = None
        self.marker_sub = self.create_subscription(
            Marker, 'target_marker', self.on_marker, 10
        )
        self.reachable_pub = self.create_publisher(Bool, 'target_reached', 10)
        self.create_timer(0.1, self.check_once)

    def on_marker(self, msg):
        self.latest_marker = msg

    def check_once(self):
        if self.latest_marker is None:
            self.get_logger().warning(
                'Waiting for target_marker -- is target_slider_gui running?'
            )
            self.reachable_pub.publish(Bool(data=False))
            return

        parent = self.latest_marker.header.frame_id
        child = self.get_parameter('child_frame').value
        tolerance = self.get_parameter('tolerance_m').value
        p = self.latest_marker.pose.position
        expected = (p.x, p.y, p.z)

        try:
            tf = self.buffer.lookup_transform(parent, child, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warning(f'TF not ready yet: {ex}')
            self.reachable_pub.publish(Bool(data=False))
            return

        t = tf.transform.translation
        actual = (t.x, t.y, t.z)
        error = math.dist(actual, expected)
        passed = error <= tolerance
        verdict = 'PASS' if passed else 'FAIL'

        self.get_logger().info(
            f'[{verdict}] {parent} -> {child}  '
            f'actual=({actual[0]:.4f}, {actual[1]:.4f}, {actual[2]:.4f})  '
            f'expected=({expected[0]:.4f}, {expected[1]:.4f}, {expected[2]:.4f})  '
            f'error={error * 1000:.1f} mm (tolerance {tolerance * 1000:.0f} mm)'
        )
        self.reachable_pub.publish(Bool(data=passed))


def main():
    rclpy.init()
    node = FKCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()