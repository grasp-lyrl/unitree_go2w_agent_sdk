#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import tf_transformations  # `pip install tf-transformations`
from builtin_interfaces.msg import Time

class PoseToTF2Node(Node):
    def __init__(self):
        super().__init__('pose_to_tf2_node')

        # Parameters
        self.declare_parameter("pose_topic", "/end_pose")
        self.declare_parameter("child_frame_id", "wrist")
        self.declare_parameter("parent_frame_id", "base_link")

        # Get parameters
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        self.parent_frame_id = self.get_parameter("parent_frame_id").get_parameter_value().string_value

        # Subscriber
        self.subscription = self.create_subscription(
            Pose,
            self.pose_topic,
            self.pose_callback,
            10
        )

        # TF Broadcaster
        self.br = TransformBroadcaster(self)
        self.get_logger().info(f"Subscribed to {self.pose_topic} and publishing TF from {self.parent_frame_id} → {self.child_frame_id}")

    def pose_callback(self, msg: Pose):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id

        # Fill in translation
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        # Fill in rotation (already in quaternion format)
        t.transform.rotation = msg.orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
