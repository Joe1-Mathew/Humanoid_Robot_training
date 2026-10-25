import tkinter as tk

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

# base_link -> hip pitch axis offset
HIP_X0 = -0.028164
HIP_Y0 = 0.072
HIP_Z0 = -0.159124

TARGET_X_RANGE = (-0.5, 0.5)
TARGET_Z_RANGE = (-0.55, 0.1)

INITIAL_TARGET_X = 0.15
INITIAL_TARGET_Z = -0.45


class TargetSliderGui(Node):
    def __init__(self):
        super().__init__('target_slider_gui')
        self.publisher = self.create_publisher(Point, 'target_command', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        self.reachable_sub = self.create_subscription(
            Bool, 'target_reached', self.on_reachable, 10
        )
        self.reachable = False  # red until something says otherwise
        self.last_x = INITIAL_TARGET_X
        self.last_z = INITIAL_TARGET_Z

    def on_reachable(self, msg):
        self.reachable = msg.data
        self.publish_marker(self.last_x, self.last_z)

    def publish_target(self, x, z):
        self.last_x, self.last_z = x, z

        msg = Point()
        msg.x = x
        msg.z = z
        self.publisher.publish(msg)

        self.publish_marker(x, z)

    def publish_marker(self, x, z):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = HIP_X0 + x
        marker.pose.position.y = HIP_Y0
        marker.pose.position.z = HIP_Z0 + z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        if self.reachable:
            marker.color.g = 1.0
        else:
            marker.color.r = 1.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = TargetSliderGui()

    root = tk.Tk()
    root.title('target sliders')

    def on_slider_move(_value=None):
        node.publish_target(x_slider.get(), z_slider.get())

    tk.Label(root, text='target_x (m), relative to the hip pitch axis').pack()
    x_slider = tk.Scale(
        root, from_=TARGET_X_RANGE[0], to=TARGET_X_RANGE[1], resolution=0.005,
        orient=tk.HORIZONTAL, length=400, command=on_slider_move,
    )
    x_slider.set(INITIAL_TARGET_X)
    x_slider.pack()

    tk.Label(root, text='target_z (m), relative to the hip pitch axis').pack()
    z_slider = tk.Scale(
        root, from_=TARGET_Z_RANGE[0], to=TARGET_Z_RANGE[1], resolution=0.005,
        orient=tk.HORIZONTAL, length=400, command=on_slider_move,
    )
    z_slider.set(INITIAL_TARGET_Z)
    z_slider.pack()

    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(50, spin_ros)

    rclpy.spin_once(node, timeout_sec=0.1)
    on_slider_move()
    root.after(50, spin_ros)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()