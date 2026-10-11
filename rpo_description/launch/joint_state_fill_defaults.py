#!/usr/bin/env python3
"""
Republishes a JointState message, filling in 0.0 for missing joint mappings to hide GUI sliders.
"""
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateFillDefaults(Node):

    def __init__(self):
        super().__init__('joint_state_fill_defaults')

        self.declare_parameter('source_topic', 'joint_states')
        self.declare_parameter('target_topic', 'filled/joint_states')
        self.declare_parameter('joint_names_file', '')

        joint_names_file = self.get_parameter('joint_names_file').value
        with open(joint_names_file, 'r') as f:
            joint_names = json.load(f)

        self.last_position = {name: 0.0 for name in joint_names}

        source_topic = self.get_parameter('source_topic').value
        target_topic = self.get_parameter('target_topic').value

        self.sub = self.create_subscription(JointState, source_topic, self.on_joint_states, 10)
        self.pub = self.create_publisher(JointState, target_topic, 10)

        self.get_logger().info(
            f"Filling defaults for '{source_topic}' -> '{target_topic}' ({len(joint_names)} joint(s))"
        )

    def on_joint_states(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.last_position and i < len(msg.position):
                self.last_position[name] = msg.position[i]

        out = JointState()
        out.header = msg.header
        out.name = list(self.last_position.keys())
        out.position = list(self.last_position.values())
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateFillDefaults()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()