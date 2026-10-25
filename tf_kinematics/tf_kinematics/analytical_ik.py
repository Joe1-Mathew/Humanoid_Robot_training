import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

HIP_PITCH_LIMITS = (-2.094, 0.7854)
KNEE_LIMITS = (-0.2, 2.5)

# Effective link lengths for the hip_pitch -> knee -> ankle
L1 = 0.25  # left_thigh_pitch_link -> left_knee_link (hip axis to knee axis)
L2 = 0.30  # left_knee_link -> left_ankle_pitch_link (knee axis to ankle axis)


def ik_leg(x, z, l1, l2, knee_forward=True):
    """Analytic law-of-cosines IK for a 2-link planar leg in the X-Z plane"""
    r = math.sqrt(x * x + z * z)

    # Reject unreachable targets (too far or too close)
    if r > (l1 + l2) or r < abs(l1 - l2):
        return None

    cos_q2 = (r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2)

    # Clamp cos_q2 to [-1, 1] to avoid math domain errors due to floating point inaccuracies
    cos_q2 = max(-1.0, min(1.0, cos_q2))

    # Choose the correct solution based on the knee_forward parameter
    if knee_forward:
        q2 = math.acos(cos_q2)
    else:
        q2 = -math.acos(cos_q2)

    q1 = math.atan2(-x, -z) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))

    return q1, q2


def fk_leg(q1, q2, l1, l2):
    """Forward kinematics for 2-link planar leg in the X-Z plane"""
    x = -l1 * math.sin(q1) - l2 * math.sin(q1 + q2)
    z = -l1 * math.cos(q1) - l2 * math.cos(q1 + q2)
    return x, z


class AnalyticalIKNode(Node):
    def __init__(self):
        super().__init__('analytical_ik')

        self.declare_parameter('knee_forward', True)

        self.joint_names = None
        self.positions = None
        self.hip_idx = None
        self.knee_idx = None
        self.reachable = False
        self.last_x = 0.15
        self.last_z = -0.45

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.reachable_pub = self.create_publisher(Bool, 'target_reached', 10)
        self.target_sub = self.create_subscription(
            Point, 'target_command', self.on_target_command, 10
        )
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.robot_description_sub = self.create_subscription(
            String, 'robot_description', self.on_robot_description, robot_description_qos
        )
        self.create_timer(0.1, self.publish_all)
        self.add_on_set_parameters_callback(self.on_parameter_change)


    def non_fixed_joint_names(self, urdf_xml):
        """Return a list of non-fixed joint names from a URDF XML string"""
        root = ET.fromstring(urdf_xml)
        return [
            joint.get('name')
            for joint in root.findall('joint')
            if joint.get('type') != 'fixed'
        ]

    def on_robot_description(self, msg):
        self.joint_names = self.non_fixed_joint_names(msg.data)
        self.positions = [0.0] * len(self.joint_names)
        self.hip_idx = self.joint_names.index('left_hip_pitch_joint')
        self.knee_idx = self.joint_names.index('left_knee_joint')
        self.get_logger().info(
            f'Got robot_description: {len(self.joint_names)} non-fixed joints.'
        )
        self.solve_and_publish_target(
            self.last_x, self.last_z, self.get_parameter('knee_forward').value
        )

    def on_target_command(self, msg):
        self.solve_and_publish_target(msg.x, msg.z, self.get_parameter('knee_forward').value)

    def on_parameter_change(self, params):
        for p in params:
            if p.name == 'knee_forward':
                self.solve_and_publish_target(self.last_x, self.last_z, p.value)
        return SetParametersResult(successful=True)

    def solve_and_publish_target(self, x, z, knee_forward):
        self.last_x, self.last_z = x, z

        if self.joint_names is None:
            self.get_logger().warning(
                'Waiting for robot_description -- is robot_state_publisher running?'
            )
            return

        branch = 'knee-forward' if knee_forward else 'knee-backward'

        solution = ik_leg(x, z, L1, L2, knee_forward=knee_forward)
        if solution is None:
            self.get_logger().error(
                f'{branch}: target ({x:.3f}, {z:.3f}) unreachable (L1={L1}, L2={L2}) '
                '-- leg holds its last valid pose'
            )
            self.reachable = False
            self.reachable_pub.publish(Bool(data=False))
            return

        q1, q2 = solution
        q1_ok = HIP_PITCH_LIMITS[0] <= q1 <= HIP_PITCH_LIMITS[1]
        q2_ok = KNEE_LIMITS[0] <= q2 <= KNEE_LIMITS[1]

        if not (q1_ok and q2_ok):
            self.get_logger().error(
                f'{branch}: hip_pitch={math.degrees(q1):.1f} deg, knee={math.degrees(q2):.1f} deg '
                'VIOLATES a real joint limit -- leg holds its last valid pose'
            )
            self.reachable = False
            self.reachable_pub.publish(Bool(data=False))
            return

        x_check, z_check = fk_leg(q1, q2, L1, L2)
        self.get_logger().info(
            f'{branch}: left_hip_pitch_joint={math.degrees(q1):.1f} deg, '
            f'left_knee_joint={math.degrees(q2):.1f} deg (within joint limits), '
            f'FK self-check -> ({x_check:.3f}, {z_check:.3f})'
        )
        self.positions[self.hip_idx] = q1
        self.positions[self.knee_idx] = q2
        self.reachable = True
        self.reachable_pub.publish(Bool(data=True))

    def publish_all(self):
        if self.joint_names is None:
            return

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.positions
        self.joint_state_pub.publish(joint_msg)


def main():
    rclpy.init()
    node = AnalyticalIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()