#!/usr/bin/env python3

"""Publish a stable demonstration pose for the Popeye dual-arm URDF."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LEFT_JOINTS = [
    "L_Base",
    "L_Shoulder",
    "L_Elbow",
    "L_Wrist1",
    "L_Wrist2",
    "L_Wrist3",
]
RIGHT_JOINTS = [
    "R_Base",
    "R_Shoulder",
    "R_Elbow",
    "R_Wrist1",
    "R_Wrist2",
    "R_Wrist3",
]
DEFAULT_POSE = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


class DemoJointStatePublisher(Node):
    """Publish configurable left and right joint positions."""

    def __init__(self) -> None:
        super().__init__("popeye_demo_joint_state_publisher")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("left_joint_positions", DEFAULT_POSE)
        self.declare_parameter("right_joint_positions", DEFAULT_POSE)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.publish_joint_state)

    def publish_joint_state(self) -> None:
        """Publish one complete 12-joint state sample."""
        left = list(self.get_parameter("left_joint_positions").value)
        right = list(self.get_parameter("right_joint_positions").value)
        if len(left) != 6 or len(right) != 6:
            self.get_logger().error(
                "left_joint_positions and right_joint_positions must contain 6 values"
            )
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = LEFT_JOINTS + RIGHT_JOINTS
        message.position = left + right
        self.publisher.publish(message)


def main(args=None) -> None:
    """Run the demonstration joint-state publisher."""
    rclpy.init(args=args)
    node = DemoJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
