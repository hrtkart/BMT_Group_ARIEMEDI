#!/usr/bin/env python3

"""Publish model TCP poses using the TF generated from the Popeye URDF."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class ModelEefPosePublisher(Node):
    """Bridge model TF poses to collision_ur's actual EEF pose topics."""

    def __init__(self) -> None:
        super().__init__("popeye_model_eef_pose_publisher")
        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.pose_publishers = {
            "left": self.create_publisher(PoseStamped, "/left_arm/eef_pose", 10),
            "right": self.create_publisher(PoseStamped, "/right_arm/eef_pose", 10),
        }
        self.frames = {
            "left": ("left_base", "L_wrist_3_link"),
            "right": ("right_base", "R_wrist_3_link"),
        }
        self.create_timer(1.0 / 30.0, self.publish_model_poses)

    def publish_model_poses(self) -> None:
        """Publish both wrist-3-centered TCP poses when TF is available."""
        for arm, (base_frame, tcp_frame) in self.frames.items():
            try:
                transform = self.buffer.lookup_transform(
                    base_frame, tcp_frame, Time()
                )
            except TransformException:
                continue
            message = PoseStamped()
            message.header = transform.header
            message.header.frame_id = base_frame
            message.pose.position.x = transform.transform.translation.x
            message.pose.position.y = transform.transform.translation.y
            message.pose.position.z = transform.transform.translation.z
            message.pose.orientation = transform.transform.rotation
            self.pose_publishers[arm].publish(message)


def main(args=None) -> None:
    """Run the model EEF pose publisher."""
    rclpy.init(args=args)
    node = ModelEefPosePublisher()
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
